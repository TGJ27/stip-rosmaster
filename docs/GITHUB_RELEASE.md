# First GitHub publication: STIP ROSMASTER v1.0.0

## Before creating the repository

1. Prefer a **Private** repository at first. The source includes a site-specific map, thermal-to-RGB camera homography, a prebuilt Jetson TensorRT engine, and robot STL meshes. Review permission to redistribute model/mesh/vendor assets before changing visibility to Public.
2. Review configuration files for serial numbers, IP addresses, credentials, locations and captured private data. The export scan did not find obvious embedded credentials, but this is **not a guarantee**.
3. Choose a license only after confirming which source/model/CAD assets you have rights to relicense. No open-source license is included by default.
4. Retain an independent backup of the working Jetson workspace. Your Git repository does not contain external inspection recordings, saved missions or ROS logs.

## Initialize and push the cleaned release (Jetson terminal)

Create a blank `stip-rosmaster` repo at https://github.com/new. Do **not** pre-create a GitHub README or `.gitignore`, because this source already has both.

**Recommended:** extract the supplied cleaned ZIP to a separate temporary folder, **not on top of the currently running Jetson workspace**. For example:

```bash
mkdir -p ~/Downloads/STIP_GITHUB_RELEASE
cd ~/Downloads
unzip STIP_ROSMASTER_GITHUB_V1.0.0.zip -d STIP_GITHUB_RELEASE
cd ~/Downloads/STIP_GITHUB_RELEASE/stip_rosmaster_ws

# Git initialization starts below; these files are the cleaned export.

git init
git branch -M main
# Configure YOUR OWN name/address; never paste account tokens into the source.
git config user.name "YOUR_NAME"
git config user.email "YOUR_EMAIL"

git add .
git status --short
git diff --cached --stat
# Verify model/mesh/map additions are intentional before committing.
git commit -m "Initial release: STIP ROSMASTER v1.0.0"

git remote add origin https://github.com/YOUR_USERNAME/stip-rosmaster.git
git push -u origin main
```

Authenticate through your Git credential manager, GitHub CLI, SSH or a scoped personal access token; GitHub HTTPS Git does not accept a normal account password. Never store a token in scripts or the repo.

If the directory **already has Git history**, do not blindly run `git init`, `git branch -M main`, or overwrite its remote. Check `git status`, `git remote -v` and consult its maintainer before merging histories.

### Tag your initial version

```bash
git tag -a v1.0.0 -m "First source release"
git push origin v1.0.0
```

This is a **source baseline**. Only publish a GitHub Release with a hardware-verification claim once the Jetson smoke checklist is completed. Do not upload private map/inspection data to a public release without approval.

## Future changes

```bash
cd ~/Documents/stip_rosmaster_ws
git status
git add src/ README.md docs/ CHANGELOG.md
git diff --cached --stat
git commit -m "Describe the tested change"
git push
```

For large runtime captures, leave them outside Git. Each model/mesh file included here is under GitHub's usual per-file size limit, but the repository contains substantial binary assets. If model snapshots change frequently, consider **Git LFS or GitHub release assets**, while keeping installation instructions synchronized. Do not switch to LFS without verifying the Jetson's clone/deployment setup.
