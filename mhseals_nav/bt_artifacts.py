"""Publish private content-addressed BT copies from snapshot-owned bytes."""
import hashlib
import os
from pathlib import Path
import stat
import uuid

from mhseals_config.model import _identity

DIRECTORY = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW


def materialize_trees(root, scope, resources):
    """Atomically publish verified XML bytes without following artifact links."""
    _identity(scope)
    directory = Path(os.path.abspath(root)) / scope / 'nav2-bt'
    fd = os.open('/', DIRECTORY)
    try:
        for part in directory.parts[1:]:
            try:
                os.mkdir(part, 0o700, dir_fd=fd)
                os.fsync(fd)
            except FileExistsError:
                pass
            child = os.open(part, DIRECTORY, dir_fd=fd)
            os.close(fd)
            fd = child
        for resource in resources.values():
            if 'content_hex' not in resource:
                continue
            content = bytes.fromhex(resource['content_hex'])
            checksum = hashlib.sha256(content).hexdigest()
            if checksum != resource['sha256']:
                raise ValueError('snapshot BT content digest mismatch')
            name = checksum + '.xml'
            destination = str(directory / name)
            if resource.get('artifact', destination) != destination:
                raise ValueError('BT artifact belongs to a different snapshot root')
            temporary = uuid.uuid4().hex + '.tmp'
            writer = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                             0o400, dir_fd=fd)
            try:
                with os.fdopen(writer, 'wb') as stream:
                    stream.write(content)
                    stream.flush()
                    os.fsync(stream.fileno())
                try:
                    os.link(temporary, name, src_dir_fd=fd, dst_dir_fd=fd,
                            follow_symlinks=False)
                except FileExistsError:
                    pass
            finally:
                os.unlink(temporary, dir_fd=fd)
            os.fsync(fd)
            reader = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=fd)
            with os.fdopen(reader, 'rb') as stream:
                info = os.fstat(stream.fileno())
                if (not stat.S_ISREG(info.st_mode) or info.st_mode & 0o222
                        or stream.read() != content):
                    raise ValueError('invalid immutable BT artifact')
            resource['artifact'] = destination
    finally:
        os.close(fd)
