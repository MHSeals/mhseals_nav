# Tests import the source tree first; extend the path so the installed, generated
# mhseals_nav.msg subpackage is still found next to it.
__path__ = __import__('pkgutil').extend_path(__path__, __name__)
