"""Package < deployment < operator startup YAML: one strict read, one merge, one identity."""
from hashlib import sha256
from pathlib import Path
from types import MappingProxyType

from .navigation_config import deep_merge, load_strict


def read(selected):
    """Return the selected file's resolved path, bytes and sha256 (a missing file raises)."""
    path = Path(selected).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    return path, raw, sha256(raw).hexdigest()


def resolve(package, deployment='', operator='', *, validate):
    """Merge the selected layers in order, each validated alone by ``validate(layer, label)``.

    An empty selection is skipped. Returns the merged values and one frozen
    {path, sha256, layer} identity per file read.
    """
    values, inputs = {}, []
    for label, selected in (('package', package), ('deployment', deployment),
                            ('operator', operator)):
        if not selected:
            continue
        path, raw, digest = read(selected)
        layer = load_strict(raw, path)
        validate(layer, label)
        deep_merge(values, layer)
        inputs.append(MappingProxyType({'path': str(path), 'sha256': digest, 'layer': label}))
    return values, inputs
