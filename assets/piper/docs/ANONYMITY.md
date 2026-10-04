# Anonymous review and publication checklist

This snapshot separates technical asset identity from author identity. Asset hierarchy, physical values, scene identifiers, numerical observations and upstream attribution are retained. Host-specific paths are replaced with relative references. PNG/JPEG identifying text, EXIF/XMP and editing metadata are removed without changing decoded pixels or color profiles.

Optional Blender authoring projects and the nested historical test ZIP are excluded from this review snapshot because they are not needed by the exported simulation scene and may contain embedded workstation history. They remain in the private original archive. The exported USD/OBJ/STL/URDF assets remain included.

The raw office-layout reference photograph is also excluded from this review copy because its background contains workplace signage and surrounding office details. Scene geometry, rendered observations and simulation textures are retained. The original photograph remains in the private source archive.

## Checks that remain with the submitting authors

- Verify that the hosting account, commit authors, emails, repository history, issues, releases and download URLs reveal no author identity to reviewers. A repository description saying “anonymous” does not hide account/history metadata.
- Inspect real reference photographs, reconstructed room appearance and branded textures for location, identifying labels, screens or equipment serial numbers. The pixel content is retained for research fidelity; visual anonymity also depends on information outside this package.
- Confirm redistribution rights for scans, robot models, textures and third-party assets. Upstream names and copyright notices are retained. This snapshot does not grant rights that the original bundle did not include.
- Test the exact reviewer-facing download in a logged-out browser. Check that large assets are actual files rather than Git LFS pointer text.
- Upload the review snapshot only; keep private audit records, old archives, environment credentials, `.git`, and personal configurations outside the supplementary archive.

## Evidence and integrity

Archived results retain their original numbers and historical hashes. `docs/bundle_manifest.json` records the current anonymous deliverable, and `tools/check_repository.py --full` verifies it. Runtime revalidation is distinct from metadata/portability checking.

Official requirements: [ICLR 2027 Author Guidelines](https://iclr.cc/Conferences/2027/AuthorGuidelines). Git asset transport: [Git LFS documentation](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage).
