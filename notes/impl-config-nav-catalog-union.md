# CI01 installed nav owner catalog union

Base nav `39295672fb457eaf38f0dc75a7641b0a14ba2f69`; exclusive branch
`impl/config-nav-catalog-union`. Core source pin
`c9c8d152be50e0cb9d88cc15656079ede45e7680`; hardware/testing context pins
`e8eab14ba2b247367405756705fffa14308a4f33` and
`063803d3a2ef9287bc423042b063ebae9ae0f863` are unchanged and not imported.

`9539003` adds the existing-type `DescriptorBundle` provider at
`mhseals_nav.owner_catalog.load_catalog(*, nav2=None, mavros=None,
consumer_versions=None)` and union tests (246 added lines). This follow-up adds
mapping-shape failure normalization and tests every required installed resource
missing/malformed, rather than leaking an upstream Nav2 list AttributeError.

One explicit provider reads actual installed ament resources for Nav2, MAVROS
and lidar and dynamically aggregates all actual family rows. Required families
must all validate before any complete bundle returns. Source bytes/hash/types,
canonical keys/aliases, source IDs, registry IDs and consumer pins are checked.
A shared registry binding is accepted only by Python callable object identity;
shared source IDs require identical metadata and bytes. Family revision strings
and source hash references survive, with deterministic aggregate descriptor hash.
No automatic discovery, plugin registry, testing import, or checkout fallback.

Optional Nav2/MAVROS selections are `(package-relative identity label, bytes)`.
The label is caller-supplied provenance, not proof of an installed filesystem
origin; no file is read at that label. Returned descriptor defaults describe
those requested source bytes, not compiled/live values. Partial selected layers
are not advertised as complete policy coverage: each actual family provider
must validate them. Lidar remains installed package-default inventory; separate
resolver origins identify runtime deployment/operator layers. The provider does
not resolve or relabel those layers. Incoming M01b policy/inheritance work is
not part of this source pin; dynamic row aggregation avoids fixed family counts.

Validation: 11 focused installed host tests passed from `/tmp` (2.40s), without
PYTHONPATH. Normal offline wheels were installed into an independent existing
container's dedicated venv; 11 focused tests passed (2.46s). Actual unpatched
ament lookup resolved that venv's installed package share and returned all three
families (32 rows at this pin; the implementation/test expectation is dynamic).
The container command explicitly sets `BASH_ENV=/dev/null`, sources only the
Jazzy environment, and runs pure imports/pytest with plugin autoload disabled.
No ROS node or operational command is invoked. Pip indexes/update checks are
explicitly disabled, with no dependency downloads/build isolation.

Exact existing lint tools in the pinned running `roboboat_dev` container:
`/home/roboboat/.cache/nav-integ-b58b6a5/venv/bin/python`, actual
`ament_flake8.main.main_with_errors(argv=[])` and
`ament_pep257.main.main(argv=['.'])` over scratch containing only both new files:
zero flake8 errors and zero pep257 errors. Default ament flake8 config SHA256
`3576fe1da3ee2d69175829a1ec70018d12d4cdb85f866b09378a343c5146beee`.
Container image: `sha256:4c29b81ffed7722267bf9eb41e1dad8d0b436f2be764c5c8a5174d4dfbfd48b3`.
`git diff --check` passes. No full suite, runtime behavior/delivery claim,
container start/create, host/network/device operations, UI/CLI, central assembly,
main/integ mutation, merge, push or subagents. Root reviews before CI02 assembly.
