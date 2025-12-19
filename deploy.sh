#!/bin/bash

# Deployment script for ROS 2 Robot Control Module documentation

set -e # Exit with nonzero exit code if anything fails

SOURCE_BRANCH="master"
TARGET_BRANCH="gh-pages"

# Pull requests and commits to other branches shouldn't try to deploy
if [ "$GITHUB_REF" != "refs/heads/$SOURCE_BRANCH" ]; then
    echo "Skipping deploy; just doing a build."
    npm run build
    exit 0
fi

# Save some useful information
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`

# Install dependencies and build the documentation
npm ci
npm run build

# Create a temporary directory for the deployment
cd build
git init
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"

# Inside this git repo we'll pretend to be on the $TARGET_BRANCH branch
git checkout -b $TARGET_BRANCH

# Add all the files needed for deployment
git add .

# Commit the "changes", but the previous commit will be non-empty
git commit -m "Deploy to GitHub Pages: ${SHA}"

# Get the deploy key by using the different environment variable
# that GitHub stores the private key in
DEPLOY_KEY_FILE=$(mktemp -t github-XXXXXX)
echo "$DEPLOY_KEY" > "$DEPLOY_KEY_FILE"

# Now that we're all set up, we can push to the $TARGET_BRANCH.
# The output from these commands should not be echoed to the terminal,
# as they may contain sensitive information.
echo "Pushing to $SSH_REPO on branch $TARGET_BRANCH"
git push "$SSH_REPO" $TARGET_BRANCH -f > /dev/null 2>&1

echo "Deploy completed successfully!"