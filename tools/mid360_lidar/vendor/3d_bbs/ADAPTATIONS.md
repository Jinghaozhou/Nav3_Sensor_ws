# Vendored CPU subset

Upstream: https://github.com/KOKIAOKI/3d_bbs, commit
`41529a34a2fb9618b5ff560fb3c2363f1615666d`, MIT license.
Original downloaded file checksums are recorded in `upstream.json`.

This is a minimal CPU source snapshot, not a complete upstream checkout.
The project compiles `bbs3d.cpp` and `voxelmaps.cpp` with
`scripts/prepare_bbs_model.py --build-only`; upstream file-I/O helpers are
retained for provenance but are not part of that build.

Local changes in the CPU header and search implementation retain spatially/
angularly separated near-best leaves, continue branch-and-bound within a
configured score band, expose timeout/peak-budget completeness, and check
leaf translations against the requested sensor-origin bounds. An incomplete
search cannot yield a ready online candidate packet. The worker protocol is
implemented in `scripts/bbs_worker.cpp`; VGICP, pose ambiguity and two-window
acceptance remain in the localization node.

Completeness refers to the configured discretized search and score band.
Occupancy scores do not certify fine-registration ranking or floor identity.
Runtime parameters and field-use instructions are maintained in the existing
project map/scanning manuals, not in this vendor directory.
