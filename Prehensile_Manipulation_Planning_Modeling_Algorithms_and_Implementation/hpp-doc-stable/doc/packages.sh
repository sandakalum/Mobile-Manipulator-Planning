#! /bin/sh

# This script generates pages for missing software from
# a template and a package name.
#
# This script incorporates meta-information about pacakges
# and should be updates to handle a new package.

# Options.
verbose=0
template="$1"
package="$2"

# Shared variables.
pkg_list=""

# Functions.
pkg ()
{
    tarname="$1"
    name="$2"
    git_url="$3"
    robotpkg_package="$4"

    pkg_list="$pkg_list $tarname"

    key=`echo "$tarname" | sed 's/-/_/g'`
    eval "name__$key='$name'"
    eval "git_url__$key='$git_url'"
    eval "robotpkg_package__$key='$robotpkg_package'"
}

usage ()
{
    echo "$0 [TEMPLATE] [PACKAGE]"
    echo "TEMPLATE is the page template (html/missing.html.in)."
    echo "PACKAGE must be the package UNIX name (tarname)."
}

# Check options.
if test x$package = x || test x$template = x; then
    usage
    exit 1
fi


# Package database.
pkg "hpp-fcl" \
    "hpp-fcl" \
    "git@github.com:humanoid-path-planner/fcl" \
    "path/hpp-fcl"

pkg "hpp-corbaserver" \
    "hpp-corbaserver" \
    "git@github.com:humanoid-path-planner/hpp-corbaserver" \
    "path/hpp-corbaserver"

pkg "hpp-core" \
    "hpp-core" \
    "git@github.com:humanoid-path-planner/hpp-core" \
    "path/hpp-core"

pkg "hpp-model" \
    "hpp-model" \
    "git@github.com:humanoid-path-planner/hpp-model" \
    "path/hpp-model"

pkg "hpp-model-urdf" \
    "hpp-model-urdf" \
    "git@github.com:humanoid-path-planner/hpp-model" \
    "path/hpp-model"

pkg "hpp-constraints" \
    "hpp-constraints" \
    "git@github.com:humanoid-path-planner/hpp-constraints" \
    "path/hpp-constraints"

pkg "hpp-manipulation" \
    "hpp-manipulation" \
    "git@github.com:humanoid-path-planner/hpp-manipulation" \
    "path/hpp-manipulation"

pkg "hpp-manipulation-corba" \
    "hpp-manipulation-corba" \
    "git@github.com:humanoid-path-planner/hpp-manipulation-corba" \
    "path/hpp-manipulation-corba"

pkg "hpp_ros" \
    "hpp_ros" \
    "git@github.com:humanoid-path-planner/hpp_ros" \
    "path/hpp_ros"

pkg "gepetto-viewer" \
    "gepetto-viewer" \
    "git@github.com:humanoid-path-planner/gepetto-viewer" \
    "graphics/gepetto-viewer"

pkg "gepetto-viewer-corba" \
    "gepetto-viewer-corba" \
    "git@github.com:humanoid-path-planner/gepetto-viewer-corba" \
    "graphics/gepetto-viewer-corba"

pkg "hpp-gepetto-viewer" \
    "hpp-gepetto-viewer" \
    "git@github.com:humanoid-path-planner/hpp-gepetto-viewer" \
    "graphics/hpp-gepetto-viewer"

# Retrieve information about wanted package.
key=`echo "$package" | sed 's/-/_/g'`
name=$(eval echo "\$name__$key")
git_url=$(eval echo "\$git_url__$key")
robotpkg_package=$(eval echo "\$robotpkg_package__$key")

# Display if verbose is enabled.
if test $verbose = 1; then
 echo "Information for package \`\`$package'':"
 echo -e "\tName: $name"
 echo -e "\tGit URL: $git_url"
 echo -e "\tRobotPkg package: $robotpkg_package"
 echo
 echo "Available packages are:"

 for pkg in $pkg_list; do
     echo "- $pkg"
 done
fi

# Generating steps instruction.
steps=""

if ! test "x$git_url" = x; then
    steps=`echo "$steps <li>"					\
	"to get the source code through <code>git</code>: "		\
	"<code>git clone $git_url</code>,"			\
	"then configure, compile and install the package.</code></li>"`
fi

if ! test "x$robotpkg_package" = x; then
    steps=`echo "$steps <li>"					\
	"install it through <code>robotpkg</code>: "		\
	"<code>cd robotpkg/$robotpkg_package;" \
	" make update</code></li>"`
fi


# Generate template page.
sed -e "s/@PACKAGE_NAME@/$name/g"		\
    -e "s/@PACKAGE_TARNAME@/$package/g"	       	\
    -e "s!@STEPS@!$steps!g"			\
    "$template"
