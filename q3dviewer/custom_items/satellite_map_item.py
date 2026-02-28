"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
Show satellite map tiles from OpenStreetMap on the XY ground plane in ENU (East-North-Up) coordinates.
Inspired by the ROS rviz_satellite plugin:
https://github.com/nobleo/rviz_satellite (distributed under Apache-2.0)
"""

import ctypes
import math
import threading
from io import BytesIO
from pathlib import Path

import numpy as np
import requests
from PIL import Image

from OpenGL.GL import *
from OpenGL.GL import shaders
from q3dviewer.base_item import BaseItem
from q3dviewer.utils import set_uniform
from q3dviewer.Qt.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
)


def _wgs_to_tile_xy(lat_deg, lon_deg, zoom):
    """WGS-84 (lat, lon) in degrees → fractional Slippy-Map tile (x, y).

    See https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    """
    n = 1 << zoom
    lat = math.radians(lat_deg)
    x = n * (lon_deg + 180.0) / 360.0
    y = n * (1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0
    return x, y


def _tile_size_meters(lat_deg, zoom):
    """Ground-truth size of one 256-px tile in metres."""
    return 156543.034 * math.cos(math.radians(lat_deg)) * 256 / (1 << zoom)


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
    """Render satellite / OSM map tiles on the XY ground plane in ENU."""

    DEFAULT_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

    def __init__(self, zoom=19, blocks=2, alpha=0.7, height=0.0, tile_url=None):
        super().__init__()
        self._zoom = int(zoom)
        self._blocks = int(blocks)
        self._alpha = float(alpha)
        self._height = float(height)         # Z offset of the map plane
        self._tile_url = tile_url or self.DEFAULT_TILE_URL

        # WGS-84 origin (set via set_origin)
        self._origin_lat = None
        self._origin_lon = None
        self._origin_alt = 0.0

        # GL resources --  (tx, ty) → {tex, vao, vbo, ebo}
        self._gl_tiles = {}
        self._program = None
        self._active_zoom = None    # zoom level of tiles currently on GPU

        # Threading / pending uploads
        self._lock = threading.Lock()
        self._pending = {}          # (tx, ty) → PIL.Image (RGBA)
        self._needed_keys = set()   # set of (tx, ty) that should be displayed
        self._need_sync = False     # True when paint() must reconcile GL tiles
        self._build_epoch = 0       # bumped on every rebuild request

        # Disk cache
        self._cache_dir = Path.home() / ".cache" / "q3dviewer_satellite"
        self._cache_dir.mkdir(parents=True, exist_ok=True)

    def set_origin(self, lat, lon, alt=0.0):
        """Set the ENU origin (degrees) and begin downloading tiles."""
        self._origin_lat = float(lat)
        self._origin_lon = float(lon)
        self._origin_alt = float(alt)
        self._request_rebuild()

    def add_setting(self, layout):
        # Origin group (Latitude & Longitude)
        origin_group = QGroupBox("Origin")
        origin_layout = QVBoxLayout()
        
        # Latitude
        self._lat_spin = QDoubleSpinBox()
        self._lat_spin.setPrefix("Lat: ")
        self._lat_spin.setRange(-90.0, 90.0)
        self._lat_spin.setDecimals(6)
        self._lat_spin.setSingleStep(0.0001)
        if self._origin_lat is not None:
            self._lat_spin.setValue(self._origin_lat)
        self._lat_spin.valueChanged.connect(self._on_origin_changed)
        origin_layout.addWidget(self._lat_spin)

        # Longitude
        self._lon_spin = QDoubleSpinBox()
        self._lon_spin.setPrefix("Lon: ")
        self._lon_spin.setRange(-180.0, 180.0)
        self._lon_spin.setDecimals(6)
        self._lon_spin.setSingleStep(0.0001)
        if self._origin_lon is not None:
            self._lon_spin.setValue(self._origin_lon)
        self._lon_spin.valueChanged.connect(self._on_origin_changed)
        origin_layout.addWidget(self._lon_spin)
        
        # Load button
        self._load_btn = QPushButton("Load Map")
        self._load_btn.clicked.connect(self._on_load)
        origin_layout.addWidget(self._load_btn)

        origin_group.setLayout(origin_layout)
        layout.addWidget(origin_group)

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

    def _on_origin_changed(self):
        self._origin_lat = self._lat_spin.value()
        self._origin_lon = self._lon_spin.value()

    def _on_load(self):
        """Load / reload the map tiles with the current origin, zoom and blocks."""
        if self._origin_lat is not None and self._origin_lon is not None:
            self._request_rebuild()

    def _on_zoom(self, v):
        if v != self._zoom:
            self._zoom = v
            self._request_rebuild()

    def _on_blocks(self, v):
        if v != self._blocks:
            self._blocks = v
            self._request_rebuild()

    def _request_rebuild(self):
        """Compute the needed tile set and download only missing tiles."""
        self._build_epoch += 1
        with self._lock:
            self._pending.clear()
        if self._origin_lat is None:
            return
        # Compute which tiles are needed at current zoom / blocks
        cx_f, cy_f = _wgs_to_tile_xy(
            self._origin_lat, self._origin_lon, self._zoom)
        cx, cy = int(cx_f), int(cy_f)
        self._needed_keys = {
            (cx + dx, cy + dy)
            for dx in range(-self._blocks, self._blocks + 1)
            for dy in range(-self._blocks, self._blocks + 1)
        }
        self._need_sync = True
        self._start_download()

    def _tile_cache_path(self, tx, ty, z):
        return self._cache_dir / f"{z}_{tx}_{ty}.png"

    def _fetch_tile_image(self, tx, ty, z):
        """Return PIL.Image (RGBA) from cache or network.  *None* on error."""
        p = self._tile_cache_path(tx, ty, z)
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
                headers={"User-Agent": "q3dviewer-satellite/1.0 (cloud_forge)"},
                timeout=15)
            r.raise_for_status()
            img = Image.open(BytesIO(r.content)).convert("RGBA")
            img.save(str(p), "PNG")
            return img
        except Exception as e:
            print(f"[SatelliteMap] tile z={z} x={tx} y={ty}: {e}")
            return None

    def _start_download(self):
        epoch = self._build_epoch
        zoom = self._zoom
        needed = set(self._needed_keys)          # snapshot
        zoom_changed = (self._active_zoom is not None
                        and self._active_zoom != zoom)
        # Tiles already on GPU that can be reused (same zoom only)
        existing = set() if zoom_changed else set(self._gl_tiles.keys())

        def _worker():
            for key in needed:
                if self._build_epoch != epoch:
                    return                              # superseded
                if key in existing:
                    continue                            # already on GPU
                tx, ty = key
                img = self._fetch_tile_image(tx, ty, zoom)
                if img is not None and self._build_epoch == epoch:
                    with self._lock:
                        self._pending[key] = img

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def initialize_gl(self):
        vs = shaders.compileShader(_VERT_SRC, GL_VERTEX_SHADER)
        fs = shaders.compileShader(_FRAG_SRC, GL_FRAGMENT_SHADER)
        self._program = shaders.compileProgram(vs, fs)

    def _make_tile_gl(self, tx, ty, pil_img):
        """Create textured quad for one tile in ENU world space."""
        lat = self._origin_lat
        lon = self._origin_lon
        zoom = self._zoom

        cx_f, cy_f = _wgs_to_tile_xy(lat, lon, zoom)
        frac_x = cx_f - math.floor(cx_f)
        frac_y = cy_f - math.floor(cy_f)
        cx, cy = int(cx_f), int(cy_f)

        dx = tx - cx
        dy = ty - cy
        ts = _tile_size_meters(lat, zoom)

        # Quad corners in ENU  (East = +X,  North = +Y)
        # OSM tile-y increases southward → flip for North axis.
        x0 = (dx - frac_x) * ts                   # west  edge
        x1 = (dx + 1 - frac_x) * ts               # east  edge
        y0 = -((dy + 1) - frac_y) * ts             # south edge
        y1 = -(dy - frac_y) * ts                   # north edge
        z = 0.0

        # Vertices: position(3) + texcoord(2)
        # Image is np.flipud'd before upload so tex (0,0) = SW.
        verts = np.array([
            x0, y0, z, 0, 0,   # SW
            x1, y0, z, 1, 0,   # SE
            x1, y1, z, 1, 1,   # NE
            x0, y1, z, 0, 1,   # NW
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
        if self._program is None or self._origin_lat is None:
            return

        # Synchronise GL tiles with the needed set
        if self._need_sync:
            self._need_sync = False
            if self._active_zoom is not None and self._active_zoom != self._zoom:
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
            batch = dict(self._pending)
            self._pending.clear()
        for (tx, ty), img in batch.items():
            if (tx, ty) not in self._gl_tiles:
                self._make_tile_gl(tx, ty, img)

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
