#!/bin/bash

set -e

# I (Joseph Mirabel) do not think there
# is a need to create tags for these ones.
# hpp-template-corba
# hpp-environments

# Version
if [ ! $# -eq 1 ]; then
  echo "usage: $0 version"
  exit 1
fi
version=$1

# Packages to tag
# pinocchio
pkg_main="hpp-util hpp-statistics hpp-fcl hpp-pinocchio \
hpp-constraints hpp-corbaserver hpp-core hpp_tutorial hpp-doc \
hpp-manipulation hpp-manipulation-corba hpp-manipulation-urdf \
hpp-gepetto-viewer hpp-gui hpp-plot"
branch="stable"
remote="origin"

c_what="\e[33m"
c_good="\e[32m"
c_bad="\e[31m"
c_reset="\e[0m"

echo "Devel directory is $DEVEL_HPP_DIR"
echo -e "${c_what}Checking that everything is good.${c_reset}"
# Check that
# - remote ${remote} is something like https://github.com/humanoid-path-planner/...
# - there is no difference between ${remote}/${branch}
#   and the current working directory.
# - working directory is clean
for pkg in ${pkg_main}; do
  GIT="git -C ${DEVEL_HPP_DIR}/src/${pkg}"

  remote_origin=$(${GIT} remote get-url ${remote})
  if [[ ! "$remote_origin" =~ ^.*github.com[:/]humanoid-path-planner/${pkg}.*$ ]]; then
    echo -e "${pkg}: ${c_bad}Remote '${remote}' is not the main repo. Should be https://github.com/humanoid-path-planner/${pkg}${c_reset}"
    # exit 1
  fi

  #current_branch=$(${GIT} rev-parse --abbrev-ref HEAD)
  #if [ ! "${current_branch}" == "${branch}" ]; then
  #  echo "${pkg}: Current branch is ${current_branch}. Should be ${branch}."
  #  exit 1
  #fi
  ${GIT} update-index -q --refresh
  CHANGED=$(${GIT} diff-index --name-only ${remote}/${branch} --)
  if [ -n "${CHANGED}" ]; then
    echo -e "${pkg}: ${c_bad}Not synchronized with ${remote}/${branch}${c_reset}."
    exit 1
  fi

  CHANGED=$(${GIT} diff-index --name-only HEAD --)
  if [ -n "${CHANGED}" ]; then
    echo -e "${pkg}: ${c_bad}Working directory not clean${c_reset}."
    exit 1
  fi
  echo -e "${pkg}: ${c_good}Ok${c_reset}."
done

# Create the tags
echo -e "${c_what}Create tags ${version}.${c_reset}"
for pkg in ${pkg_main}; do
  GIT="git -C ${DEVEL_HPP_DIR}/src/${pkg}"
  ${GIT} tag -a -m "Release of version ${version}" ${version}
  echo -e "${pkg}: ${c_good}Ok${c_reset}."
done

# Push the tags
read -p "Push the tags ? [y/N]" answer
if [[ "${answer}" =~ ^[yY]$ ]]; then
  echo -e "${c_what}Push tags ${version} to ${remote}.${c_reset}"
  for pkg in ${pkg_main}; do
    GIT="git -C ${DEVEL_HPP_DIR}/src/${pkg}"
    ${GIT} push --quiet ${remote} ${version}
    echo -e "${pkg}: ${c_good}Ok${c_reset}."
  done
fi
