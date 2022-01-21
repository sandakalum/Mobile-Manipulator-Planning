## Make a tarball

To avoid conflicts, you must setup a clean environment,
i.e. without having sourced the ROS `setup.bash` or the HPP `config.sh`.
To create the tarball, HPP will be compiled in `$DEVEL_HPP_DIR` (defaults to `/local/devel/hpp`).
You may select the type of build you want using the environment variable `BUILD_TYPE`.

If you sourced `config.sh`, this should be sufficient (although it is unsafe) to create a clean
environment:
```bash
export DEVEL_CONFIG="noconfig"
unset PATH PYTHONPATH ROS_PACKAGE_PATH LD_LIBRARY_PATH CPATH DEVEL_HPP_DIR PKG_CONFIG_PATH CMAKE_PREFIX_PATH HPPCD_PATH
/bin/bash
```

Build the tarball using
```bash
auto-install-hpp.sh --mktar -v
```

You will find three files in the directory `${DEVEL_HPP_DIR}/tarball/`:
* a bash script called `check.***.sh` that should be used in order to resolve the dependencies on a new computer,
* a tarball called `hpp.***.tar.gz` containing only the binaries,
* a tarball called `hpp.src.***.tar.gz` containing the binaries and the source files.

## Build docker images for CI

```
for ubuntu in 20.04 18.04
do
    docker build -t gitlab.laas.fr:4567/humanoid-path-planner/hpp-doc:$ubuntu -f .dockers/ubuntu-$ubuntu/Dockerfile ./scripts
    docker push gitlab.laas.fr:4567/humanoid-path-planner/hpp-doc:$ubuntu
done
```

## Make new releases

The explanation below assumes you have an account on https://gitlab.laas.fr and that
you have write-access on https://gitlab.laas.fr/humanoid-path-planner/hpp-doc.

### Naming the release

In the following, *M*, *m* and *r* correspond to major, minor and revision numbers.

- Tags *vM.m.r-rc* correspond to tested releases.
- Tags *vM.m.r* correspond to benchmarked releases.
- *stable* branches always point to the latest benchmarked release.
- *devel* branches are always based on the latest tested release.
- Major and minor numbers *M* and *m* increments are synchronized in all packages.
- Revision number *r* increment is package-wise.
  It is kept for changes which affect only one package and that do not change the
  *tested* status (i.e. compilation + unit-test).

### Tested release

The procedure only checks that the code compiles and the unittests pass.

When you are ready with your changes:
1. If hpp-doc has changes, push to branch *devel* on https://gitlab.laas.fr/humanoid-path-planner/hpp-doc
   Otherwise, trigger a new build on https://gitlab.laas.fr/humanoid-path-planner/hpp-doc/pipelines
2. If the build is successful, run `scripts/create-tags.sh <version>`.
   The script will check that:
   - You are on the *devel* branch in all hpp repositories.
   - All hpp repositories are clean (no local changes).
3. Push the tags to github.

### Benchmarked release

Benchmarked release are tested release for which the benchmark have been run.

1. Same as step 1 above, using branch *future* instead of *devel*.
2. Run the scripts in `hpp_benchmark`.
3. Same as step 2 above.
4. Move the *stable* branch to the current tag and push *stable* and tag to github. (TODO add a script)
