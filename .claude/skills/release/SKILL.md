---
name: release
description: Cut a new skelarm release following RELEASING.md — bump the version, commit, tag, and push so GitHub Actions creates the draft release and publishes to PyPI. Usage: /release <version> (e.g. /release 0.2.0).
disable-model-invocation: true
---

Release a new version of `skelarm`. The target version comes from `$ARGUMENTS` (e.g. `0.2.0`, with or without a leading `v`). If no version is given, ask for it before proceeding.

This skill automates steps 1–3 of `RELEASING.md` (version bump → commit → tag → push). The GitHub Release and PyPI publish are deliberately left to a human + GitHub Actions — see the hand-off step for why.

## Pre-flight (catch problems before anything is written)

Each check below exists to fail *early and locally* rather than mid-push, when half the work is already done and a tag may be dangling.

1. **Normalize the version**: strip any leading `v`, and confirm it looks like semver (`X.Y.Z`). Read the current `version` from `pyproject.toml`; if the target is not strictly greater, stop and ask the user to confirm — a non-increasing version is almost always a mistake.
2. **Tag must not already exist**: run `git tag -l "v<X.Y.Z>"`. If it returns the tag, stop — re-releasing an existing tag won't re-trigger the workflow cleanly and signals a mistake in the version argument.
3. **Sync with remote**: `git fetch origin`, then confirm the local branch is `main` and not behind `origin/main` (`git rev-list --left-right --count origin/main...HEAD`). Pushing from a behind branch gets rejected; better to find out now.
4. **Quality gate**: run `make all` (format + type-check + test). Run this *before* asserting a clean tree — `make format` can rewrite files, so checking cleanliness first would be misleading. If anything fails, stop.
5. **Clean tree**: after `make all`, confirm `git status` is clean. Any leftover changes mean either the formatter touched something (commit or review it first) or there's unfinished work — either way, don't proceed.
6. **Confirm**: show the user the current → new version and the exact commit/tag you're about to create. Wait for confirmation before writing anything.

## Release steps

7. **Bump version**: edit the `version` field under `[project]` in `pyproject.toml` to the new version — no `v` prefix in the file.
8. **Commit**: `git add pyproject.toml && git commit -m "chore: Bump version to v<X.Y.Z>"`.
9. **Tag and push**: create an annotated tag `git tag -a "v<X.Y.Z>" -m "v<X.Y.Z>"`, then push the commit and the tag: `git push origin main && git push origin "v<X.Y.Z>"`.

## Hand-off

10. Tell the user the tag push triggers `create-release-draft.yml`, which drafts a GitHub Release with auto-generated notes. They must review and **publish that draft on GitHub** to trigger `release.yml`, which builds and uploads to PyPI via Trusted Publishing.

Stop and report after the tag is pushed. Do not publish the GitHub release or upload to PyPI yourself: the draft is an intentional human review point, and PyPI publishing is wired to the "release published" event via Trusted Publishing — there's no token to push with manually, and doing it by hand would bypass the review gate.
