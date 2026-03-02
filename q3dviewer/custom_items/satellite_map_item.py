"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
Show satellite map tiles from OpenStreetMap on the XY ground plane in ENU (East-North-Up) coordinates.
Inspired by the ROS rviz_satellite plugin:
https://github.com/nobleo/rviz_satellite (distributed under Apache-2.0)
"""
import math
import ctypes
import threading
from io import BytesIO
from pathlib import Path
import numpy as np
import requests
from PIL import Image
from pyproj import CRS, Transformer
from OpenGL.GL import *
from OpenGL.GL import shaders
from q3dviewer.utils import set_uniform
from q3dviewer.base_item import BaseItem
from q3dviewer.Qt.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider, QSpinBox, QVBoxLayout


def lonlat_to_tile(lon_deg, lat_deg, zoom):
    """WGS-84 (lon, lat) in degrees → fractional Slippy-Map tile (x, y).

    See https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    """
    n = 1 << zoom
    lat = math.radians(lat_deg)
    x = n * (lon_deg + 180.0) / 360.0
    y = n * (1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0
    return x, y


def tile_bounds_lonlat(tx, ty, zoom):
    """Return (west_lon, south_lat, east_lon, north_lat) in WGS-84 degrees for a Slippy-Map tile."""
    n = 1 << zoom
    west_lon = tx / n * 360.0 - 180.0
    east_lon = (tx + 1) / n * 360.0 - 180.0
    north_lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    south_lat = math.degrees(
        math.atan(math.sinh(math.pi * (1 - 2 * (ty + 1) / n))))
    return west_lon, south_lat, east_lon, north_lat


_VERT_SRC = """
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;

uniform mat4  view;
uniform mat4  projection;
uniform float uHeight;

out vec2 vTexCoord;

void main() {
    vec3 pos    = aPos + vec3(0.0, 0.0, uHeight);
    vTexCoord   = aTexCoord;
    gl_Position = projection * view * vec4(pos, 1.0);
}
"""

_FRAG_SRC = """
#version 330 core
in  vec2 vTexCoord;
out vec4 FragColor;

uniform sampler2D uTexture;
uniform float     uAlpha;

void main() {
    vec4 c = texture(uTexture, vTexCoord);
    FragColor = vec4(c.rgb, c.a * uAlpha);
}
"""


class SatelliteMapItem(BaseItem):
    """Render satellite / OSM map tiles aligned with point cloud data.

    Supports both local ENU coordinates and projected CRS (JPRCS, UTM, etc.).
    Uses pyproj transformers to convert OSM tile boundaries to the target
    coordinate system for accurate alignment.
    """

    DEFAULT_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

    def __init__(self, zoom=18, blocks=2, alpha=0.7, height=0.0, tile_url=None):
        super().__init__()
        self._zoom = int(zoom)
        self._blocks = int(blocks)
        self._alpha = float(alpha)
        self._height = float(height)
        self._tile_url = tile_url or self.DEFAULT_TILE_URL

        # Map origin in local coordinate system (x, y)
        # For ENU mode: (0, 0) at lat/lon origin
        # For CRS mode: projected coordinates (easting, northing)
        self._origin_x = None
        self._origin_y = None
        self._origin_alt = 0.0

        # Coordinate system transformers (always set after calling set_origin)
        self.xy_2_lonlat = None   # target CRS → WGS-84
        self.lonlat_2_xy = None  # WGS-84 → target CRS

        # GL resources --  (tx, ty) → {tex, vao, vbo, ebo}
        self._gl_tiles = {}
        self._program = None
        self._active_zoom = None    # zoom level of tiles currently on GPU

        # Threading / pending uploads
        self._lock = threading.Lock()
        self._pending_img = {}          # (tx, ty) → PIL.Image (RGBA)
        self._needed_keys = set()   # set of (tx, ty) that should be displayed
        self._need_sync = False     # True when paint() must reconcile GL tiles
        self._build_epoch = 0       # bumped on every rebuild request

        # Disk cache
        self._cache_dir = Path.home() / ".cache" / "q3dviewer_satellite"
        self._cache_dir.mkdir(parents=True, exist_ok=True)

    def set_origin(self, lat=None, lon=None, alt=0.0, *,
                   x=None, y=None, crs=None):
        """Set the map origin and begin downloading tiles.

        Two modes:
          1) WGS-84 (ENU):  set_origin(lat=35.0, lon=139.8)
          2) Projected CRS: set_origin(x=-12345, y=67890, crs=6677)
             lat/lon are computed automatically; tiles are placed in
             source-CRS coordinates so they align with raw point clouds.

        Parameters
        ----------
        lat, lon : float | None
            WGS-84 degrees.  Required when *crs* is not given.
        alt : float
            Altitude / Z offset (metres).
        x, y : float | None
            Easting / northing in *crs*.  Required when *crs* is given.
        crs : pyproj.CRS | int (EPSG) | str (WKT / proj-string) | None
            Projected CRS of the point-cloud data.
        """
        # Setup CRS and transformers first
        if crs is not None:
            # CRS mode: use provided projected coordinates
            if x is None or y is None:
                raise ValueError("x and y are required when crs is given")
            self._setup_crs(crs)
            self._origin_x = float(x)
            self._origin_y = float(y)
        else:
            # ENU mode: create local tangent plane at lat/lon
            if lat is None or lon is None:
                raise ValueError(
                    "lat and lon are required, or provide x, y and crs")
            enu_crs = CRS.from_proj4(
                f"+proj=aeqd +lat_0={lat} +lon_0={lon} "
                f"+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")
            self._setup_crs(enu_crs)
            self._origin_x = 0.0  # ENU origin is at (0, 0)
            self._origin_y = 0.0

        self._origin_alt = float(alt)
        self.request_download()

    # ------------------------------------------------------------------ #
    #  Projected-CRS helpers (JPRCS, UTM, etc.)                          #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _extract_crs_from_vlrs(vlrs):
        """Try to build a *pyproj.CRS* from LAS GeoTIFF VLR records."""
        # 1) ESRI PE String (WKT) inside GeoAsciiParams
        for vlr in vlrs:
            for s in getattr(vlr, 'strings', []):
                if 'ESRI PE String' in s:
                    wkt = s.split('=', 1)[1].strip()
                    if wkt:
                        try:
                            return CRS.from_wkt(wkt)
                        except Exception:
                            pass
        # 2) ProjectedCSTypeGeoKey (3072) in GeoKeyDirectory
        for vlr in vlrs:
            for gk in getattr(vlr, 'geo_keys', []):
                gk_id = getattr(gk, 'id', None)
                val = getattr(gk, 'value_offset', None)
                if gk_id == 3072 and val and val not in (0, 32767):
                    try:
                        return CRS.from_epsg(val)
                    except Exception:
                        pass
        # 3) GeographicTypeGeoKey (2048)
        for vlr in vlrs:
            for gk in getattr(vlr, 'geo_keys', []):
                gk_id = getattr(gk, 'id', None)
                val = getattr(gk, 'value_offset', None)
                if gk_id == 2048 and val and val not in (0, 32767):
                    try:
                        return CRS.from_epsg(val)
                    except Exception:
                        pass
        return None

    def _setup_crs(self, crs):
        """Configure pyproj transformers for the given CRS."""
        if not isinstance(crs, CRS):
            crs = CRS(crs)
        self.xy_2_lonlat = Transformer.from_crs(
            crs, CRS.from_epsg(4326), always_xy=True)
        self.lonlat_2_xy = Transformer.from_crs(
            CRS.from_epsg(4326), crs, always_xy=True)

    def set_crs_from_file(self, file_path, center=None):
        crs, bbox_cx, bbox_cy = self.read_crs_from_file(file_path)
        
        if crs is None:
            return False
        
        # Use provided center or fall back to bbox center from file
        if center is not None:
            x, y = center[0], center[1]
        elif bbox_cx is not None and bbox_cy is not None:
            x, y = bbox_cx, bbox_cy
        else:
            return False
        
        # Set origin using the extracted CRS and coordinates
        self.set_origin(x=x, y=y, crs=crs, alt=self._origin_alt)
        
        # Switch to CRS mode
        if hasattr(self, '_mode_combo'):
            self._mode_combo.setCurrentIndex(1)
        
        # Set CRS input fields for UI
        if hasattr(self, '_epsg_spin') and crs.is_projected and crs.to_epsg():
            self._epsg_spin.setValue(crs.to_epsg())
        if hasattr(self, '_x_spin') and bbox_cx is not None:
            self._x_spin.setValue(bbox_cx if center is None else center[0])
        if hasattr(self, '_y_spin') and bbox_cy is not None:
            self._y_spin.setValue(bbox_cy if center is None else center[1])
        
        # Update lon/lat display in ENU mode fields
        if hasattr(self, '_lon_spin') and hasattr(self, '_lat_spin') and self.xy_2_lonlat is not None:
            lon, lat = self.xy_2_lonlat.transform(
                self._origin_x, self._origin_y)
            self._lon_spin.setValue(lon)
            self._lat_spin.setValue(lat)
        return True
                
    @staticmethod
    def read_crs_from_file(file_path):
        file_path = str(file_path)
        
        # Handle LAS/LAZ files
        if file_path.lower().endswith(('.las', '.laz')):
            import laspy
            with laspy.open(file_path) as reader:
                header = reader.header
                vlrs = header.vlrs
                
                # Extract bounding box center
                try:
                    bbox_cx = (header.x_min + header.x_max) / 2
                    bbox_cy = (header.y_min + header.y_max) / 2
                except Exception:
                    bbox_cx = bbox_cy = None
                
                # Extract CRS from VLRs
                crs = SatelliteMapItem._extract_crs_from_vlrs(vlrs)
                return crs, bbox_cx, bbox_cy
        # Add support for other file formats here in the future
        # elif file_path.lower().endswith('.shp'):
        #     ...
        else:
            return None, None, None

    def add_setting(self, layout):
        # ---- Visibility checkbox ----
        self._visible_checkbox = QCheckBox("Show Satellite Map")
        self._visible_checkbox.setChecked(self.visible())
        self._visible_checkbox.stateChanged.connect(
            lambda state: self.set_visible(bool(state)))
        layout.addWidget(self._visible_checkbox)

        # ---- Manual origin input ----
        origin_group = QGroupBox("Set Origin")
        origin_layout = QVBoxLayout()

        # Load from LAS file button
        self._load_las_btn = QPushButton("Load from LAS File \u2026")
        self._load_las_btn.clicked.connect(self._on_load_las_crs)
        origin_layout.addWidget(self._load_las_btn)

        # Mode selector
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItem("ENU")
        self._mode_combo.addItem("CRS")
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo)
        origin_layout.addLayout(mode_layout)

        # ENU mode inputs (lon, lat)
        self._enu_widget = QGroupBox("ENU Mode")
        enu_layout = QVBoxLayout()

        self._lon_spin = QDoubleSpinBox()
        self._lon_spin.setPrefix("Lon: ")
        self._lon_spin.setRange(-180.0, 180.0)
        self._lon_spin.setDecimals(6)
        self._lon_spin.setSingleStep(0.0001)
        self._lon_spin.setValue(0.0)
        enu_layout.addWidget(self._lon_spin)

        self._lat_spin = QDoubleSpinBox()
        self._lat_spin.setPrefix("Lat: ")
        self._lat_spin.setRange(-90.0, 90.0)
        self._lat_spin.setDecimals(6)
        self._lat_spin.setSingleStep(0.0001)
        self._lat_spin.setValue(0.0)
        enu_layout.addWidget(self._lat_spin)

        self._enu_widget.setLayout(enu_layout)
        origin_layout.addWidget(self._enu_widget)

        # CRS mode inputs (EPSG code, x, y)
        self._crs_widget = QGroupBox("CRS Mode")
        crs_layout = QVBoxLayout()

        self._epsg_spin = QSpinBox()
        self._epsg_spin.setPrefix("EPSG: ")
        self._epsg_spin.setRange(1000, 99999)
        self._epsg_spin.setValue(6677)
        crs_layout.addWidget(self._epsg_spin)

        self._x_spin = QDoubleSpinBox()
        self._x_spin.setPrefix("X: ")
        self._x_spin.setRange(-1e8, 1e8)
        self._x_spin.setDecimals(2)
        self._x_spin.setSingleStep(100.0)
        self._x_spin.setValue(0.0)
        crs_layout.addWidget(self._x_spin)

        self._y_spin = QDoubleSpinBox()
        self._y_spin.setPrefix("Y: ")
        self._y_spin.setRange(-1e8, 1e8)
        self._y_spin.setDecimals(2)
        self._y_spin.setSingleStep(100.0)
        self._y_spin.setValue(0.0)
        crs_layout.addWidget(self._y_spin)

        self._crs_widget.setLayout(crs_layout)
        origin_layout.addWidget(self._crs_widget)

        # Load button
        self._load_btn = QPushButton("Load Map")
        self._load_btn.clicked.connect(self._on_load)
        origin_layout.addWidget(self._load_btn)

        origin_group.setLayout(origin_layout)
        layout.addWidget(origin_group)

        # Set initial mode visibility
        self._on_mode_changed(0)

        # Zoom level
        self._zoom_spin = QSpinBox()
        self._zoom_spin.setPrefix("Zoom: ")
        self._zoom_spin.setRange(1, 19)
        self._zoom_spin.setValue(self._zoom)
        self._zoom_spin.valueChanged.connect(self._on_zoom)
        layout.addWidget(self._zoom_spin)

        # Blocks around centre
        self._blocks_spin = QSpinBox()
        self._blocks_spin.setPrefix("Blocks: ")
        self._blocks_spin.setRange(0, 10)
        self._blocks_spin.setValue(self._blocks)
        self._blocks_spin.valueChanged.connect(self._on_blocks)
        layout.addWidget(self._blocks_spin)

        # Opacity
        h3 = QHBoxLayout()
        h3.addWidget(QLabel("Alpha:"))
        self._alpha_slider = QSlider()
        self._alpha_slider.setOrientation(1)          # Qt.Horizontal
        self._alpha_slider.setRange(0, 100)
        self._alpha_slider.setValue(int(self._alpha * 100))
        self._alpha_slider.valueChanged.connect(
            lambda v: setattr(self, '_alpha', v / 100.0))
        h3.addWidget(self._alpha_slider)
        layout.addLayout(h3)

        # Map plane height (Z offset)
        self._height_spin = QDoubleSpinBox()
        self._height_spin.setPrefix("Height: ")
        self._height_spin.setRange(-1000.0, 1000.0)
        self._height_spin.setSingleStep(0.5)
        self._height_spin.setDecimals(1)
        self._height_spin.setValue(self._height)
        self._height_spin.valueChanged.connect(
            lambda v: setattr(self, '_height', v))
        layout.addWidget(self._height_spin)

    def _on_mode_changed(self, index):
        """Toggle visibility between ENU and CRS input widgets."""
        is_enu = (index == 0)
        self._enu_widget.setVisible(is_enu)
        self._crs_widget.setVisible(not is_enu)

    def _on_load_las_crs(self):
        """Open a file dialog, parse LAS VLRs, and configure the CRS."""
        path, _ = QFileDialog.getOpenFileName(
            None, "Select LAS File", "",
            "LAS Files (*.las *.laz);;All Files (*)")
        if not path:
            return
        
        self.set_crs_from_file(path)

    def _on_load(self):
        """Load / reload the map tiles with the current origin, zoom and blocks."""
        if self._mode_combo.currentIndex() == 0:
            # ENU mode
            lat = self._lat_spin.value()
            lon = self._lon_spin.value()
            self.set_origin(lat=lat, lon=lon, alt=self._origin_alt)
        else:
            # CRS mode
            epsg = self._epsg_spin.value()
            x = self._x_spin.value()
            y = self._y_spin.value()
            try:
                self.set_origin(x=x, y=y, crs=epsg, alt=self._origin_alt)
            except Exception as e:
                print(f"Failed to set origin: {e}")

    def _on_zoom(self, v):
        if v != self._zoom:
            self._zoom = v
            self.request_download()

    def _on_blocks(self, v):
        if v != self._blocks:
            self._blocks = v
            self.request_download()

    def request_download(self):
        """Compute the needed tile set and download only missing tiles."""
        self._build_epoch += 1
        with self._lock:
            self._pending_img.clear()
        if self._origin_x is None or self.xy_2_lonlat is None:
            return

        # get the center tile coordinates from the current origin
        center_lon, center_lat = self.xy_2_lonlat.transform(
            self._origin_x, self._origin_y)

        # Compute which tiles are needed at current zoom / blocks
        cx_f, cy_f = lonlat_to_tile(center_lon, center_lat, self._zoom)
        cx, cy = int(cx_f), int(cy_f)
        self._needed_keys = {
            (cx + dx, cy + dy)
            for dx in range(-self._blocks, self._blocks + 1)
            for dy in range(-self._blocks, self._blocks + 1)
        }
        self.start_background_download()
        self._need_sync = True

    def fetch_tile_image(self, tx, ty, z):
        """Return PIL.Image (RGBA) from cache or network.  *None* on error."""
        p = self._cache_dir / f"{z}_{tx}_{ty}.png"

        if p.exists():
            try:
                return Image.open(p).convert("RGBA")
            except Exception:
                p.unlink(missing_ok=True)

        url = (self._tile_url
               .replace("{z}", str(z))
               .replace("{x}", str(tx))
               .replace("{y}", str(ty)))
        try:
            r = requests.get(
                url,
                headers={
                    "User-Agent": "q3dviewer-satellite/1.0 (cloud_forge)"},
                timeout=15)
            r.raise_for_status()
            img = Image.open(BytesIO(r.content)).convert("RGBA")
            img.save(str(p), "PNG")
            return img
        except Exception as e:
            print(f"[SatelliteMap] tile z={z} x={tx} y={ty}: {e}")
            return None

    def start_background_download(self):
        epoch = self._build_epoch
        zoom = self._zoom
        needed = set(self._needed_keys)          # snapshot
        # Tiles already on GPU that can be reused (same zoom only)
        existing = set(self._gl_tiles.keys())
        zoom_changed = (self._active_zoom is not None
                        and self._active_zoom != zoom)
        if zoom_changed:
            existing = set()  # Zoom changed → all existing tiles are invalid

        def _worker():
            for key in needed:
                if self._build_epoch != epoch:
                    return
                if key in existing:
                    continue
                tx, ty = key
                img = self.fetch_tile_image(tx, ty, zoom)
                if img is not None and self._build_epoch == epoch:
                    with self._lock:
                        self._pending_img[key] = img

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def initialize_gl(self):
        vs = shaders.compileShader(_VERT_SRC, GL_VERTEX_SHADER)
        fs = shaders.compileShader(_FRAG_SRC, GL_FRAGMENT_SHADER)
        self._program = shaders.compileProgram(vs, fs)

    def make_tile_gl(self, tx, ty, pil_img):
        """Create textured quad for one tile in world space."""
        zoom = self._zoom
        z = 0.0

        # 1. Get tile boundary in lon/lat
        west_lon, south_lat, east_lon, north_lat = tile_bounds_lonlat(
            tx, ty, zoom)

        # 2. Transform to target coordinate system
        x_sw, y_sw = self.lonlat_2_xy.transform(west_lon, south_lat)
        x_se, y_se = self.lonlat_2_xy.transform(east_lon, south_lat)
        x_ne, y_ne = self.lonlat_2_xy.transform(east_lon, north_lat)
        x_nw, y_nw = self.lonlat_2_xy.transform(west_lon, north_lat)

        verts = np.array([
            x_sw, y_sw, z, 0, 0,   # SW
            x_se, y_se, z, 1, 0,   # SE
            x_ne, y_ne, z, 1, 1,   # NE
            x_nw, y_nw, z, 0, 1,   # NW
        ], dtype=np.float32)

        idx = np.array([0, 1, 2, 0, 2, 3], dtype=np.uint32)

        # ---- texture ----
        tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)

        w, h = pil_img.size
        data = np.flipud(np.array(pil_img)).tobytes()
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, data)
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)

        # ---- VAO / VBO / EBO ----
        vao = glGenVertexArrays(1)
        glBindVertexArray(vao)

        vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, verts.nbytes, verts, GL_STATIC_DRAW)
        stride = 5 * 4                                     # 5 floats × 4 bytes
        glEnableVertexAttribArray(0)                        # aPos
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              stride, ctypes.c_void_p(0))
        glEnableVertexAttribArray(1)                        # aTexCoord
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                              stride, ctypes.c_void_p(12))

        ebo = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.nbytes, idx, GL_STATIC_DRAW)

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        self._gl_tiles[(tx, ty)] = {
            'tex': tex, 'vao': vao, 'vbo': vbo, 'ebo': ebo,
        }

    def _delete_tile_gl(self, key):
        """Free GL resources for a single tile and remove it from the dict."""
        d = self._gl_tiles.pop(key, None)
        if d is None:
            return
        glDeleteTextures([d['tex']])
        glDeleteVertexArrays(1, [d['vao']])
        glDeleteBuffers(1, [d['vbo']])
        glDeleteBuffers(1, [d['ebo']])

    def _delete_all_gl_tiles(self):
        for d in self._gl_tiles.values():
            glDeleteTextures([d['tex']])
            glDeleteVertexArrays(1, [d['vao']])
            glDeleteBuffers(1, [d['vbo']])
            glDeleteBuffers(1, [d['ebo']])
        self._gl_tiles.clear()

    # ---- main render entry point ----

    def paint(self):
        if self._program is None or self._origin_x is None:
            return

        # Synchronise GL tiles with the needed set
        if self._need_sync:
            self._need_sync = False
            zoom_changed = (self._active_zoom is not None
                            and self._active_zoom != self._zoom)
            if zoom_changed:
                # Zoom changed → all old tiles are invalid
                self._delete_all_gl_tiles()
            else:
                # Same zoom → keep tiles still in range, delete the rest
                stale = [k for k in self._gl_tiles if k not in self._needed_keys]
                for k in stale:
                    self._delete_tile_gl(k)
            self._active_zoom = self._zoom

        # Upload freshly downloaded tiles that arrived from the worker thread
        with self._lock:
            batch = dict(self._pending_img)
            self._pending_img.clear()
        for (tx, ty), img in batch.items():
            if (tx, ty) not in self._gl_tiles:
                self.make_tile_gl(tx, ty, img)

        if not self._gl_tiles:
            return

        # ---- draw all tiles ----
        glUseProgram(self._program)

        set_uniform(self._program,
                    self.glwidget().view_matrix, 'view')
        set_uniform(self._program,
                    self.glwidget().projection_matrix, 'projection')
        set_uniform(self._program, self._alpha, 'uAlpha')
        set_uniform(self._program, self._height, 'uHeight')
        glUniform1i(glGetUniformLocation(self._program, 'uTexture'), 0)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glActiveTexture(GL_TEXTURE0)

        for d in self._gl_tiles.values():
            glBindTexture(GL_TEXTURE_2D, d['tex'])
            glBindVertexArray(d['vao'])
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, None)

        glBindVertexArray(0)
        glBindTexture(GL_TEXTURE_2D, 0)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
        glUseProgram(0)
