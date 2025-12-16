# Releasing `skelarm`

This document outlines the procedure for releasing new versions of the `skelarm` library. The process leverages Git tags and GitHub Actions to automate draft release creation and PyPI publication.

## Release Procedure

Follow these steps to prepare and publish a new version:

### 1. Update the Version Number

1.  **Edit `pyproject.toml`**: Manually update the `version` field in the `[project]` section of `pyproject.toml` to the new target version (e.g., `0.2.0`).

    ```toml
    # pyproject.toml
    [project]
    name = "skelarm"
    version = "0.2.0" # <-- Update this line
    # ...
    ```

### 2. Commit the Version Update

Create a dedicated commit for this version change. It's good practice to have a commit specifically for version updates.

```bash
git add pyproject.toml
git commit -m "chore: Bump version to v0.2.0" # Use your new version
```

### 3. Create and Push the Version Tag

Create an annotated Git tag matching the new version number, prefixed with `v` (e.g., `v0.2.0`). Then, push both the commit and the new tag to your GitHub repository.

```bash
git tag v0.2.0              # Use your new version
git push origin main        # Assuming 'main' is your primary branch
git push origin v0.2.0      # Push the new tag
```

### 4. GitHub Actions Workflow Triggered (Automatic)

Upon pushing the `vX.Y.Z` tag:

*   The `.github/workflows/create-release-draft.yml` workflow will automatically run.
*   This workflow uses `gh release create` to create a **Draft Release** on your GitHub repository (under the "Releases" section). It will have the tag name as its title and automatically generated release notes summarizing changes since the last tag.

### 5. Review and Publish the Release (Manual on GitHub)

1.  Navigate to your GitHub repository's **"Releases"** page (`https://github.com/hrshtst/skelarm/releases`).
2.  Find the newly created draft release (e.g., "Draft: v0.2.0").
3.  Click "Edit" to review the auto-generated release notes. You can refine them, add more details, or attach any manual assets (though PyPI artifacts are handled automatically).
4.  Once you are satisfied with the release notes and details, click the **"Publish release"** button.

### 6. PyPI Publication Triggered (Automatic)

*   Publishing the release on GitHub will trigger the `.github/workflows/release.yml` workflow.
*   This workflow will build your package using `uv` and automatically upload the distribution artifacts (wheel and source tarball) to [PyPI](https://pypi.org/project/skelarm/).

---

**Important Note:** Ensure you have configured PyPI Trusted Publishing in your PyPI account for the `release.yml` workflow to succeed.
