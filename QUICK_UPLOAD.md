# Quick Upload Guide

## Upload to TestPyPI (Testing)

```bash
# 1. Clean and build
rm -rf dist/ build/ *.egg-info
python3 -m build

# 2. Check package
twine check dist/*

# 3. Upload (source distribution only)
twine upload --repository testpypi dist/*.tar.gz

# 4. Test install
pip install --index-url https://test.pypi.org/simple/ \
    --extra-index-url https://pypi.org/simple/ \
    q3dviewer
```

## Upload to PyPI (Production)

```bash
# 1. Update version in pyproject.toml (remove .dev0)
# version = "1.3.2"

# 2. Build
rm -rf dist/ build/ *.egg-info
python3 -m build

# 3. Upload
twine upload dist/*.tar.gz

# 4. Create git tag
git tag -a v1.3.2 -m "Release 1.3.2"
git push origin v1.3.2
```

## Notes

- **Only upload `.tar.gz`** (source distribution)
- Platform-specific wheels (`linux_x86_64`) are rejected by PyPI
- CUDA extension compiles automatically during user installation if toolkit available
- Username: `__token__`
- Password: Your PyPI API token

## Get API Token

- TestPyPI: https://test.pypi.org/manage/account/
- PyPI: https://pypi.org/manage/account/
