# push all packages that have HPP_REPO as the repository to HPP_REPO
push:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".push; \
	done

# push tags corresponding to package versions for all packages that have
# HPP_REPO as the repository to HPP_REPO
push-tags:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".push-tag; \
	done

# Remove tags corresponding to package versions  for all packages that have
# HPP_REPO as the repository
remove-tags:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".remove-tag; \
	done

# Print version as retrieved by pkg-config for all packages
print-version:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".print-version; \
	done

# Make release in all package that have variable ${package}_version defined
release:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		${MAKE} "$$child_dir".release; \
	done


# if repository is HPP_REPO, push current HEAD in master branch
%.push:
	cd ${SRC_DIR}/$(@:.push=); \
	if [ "${$(@:.push=)_repository}" = "${HPP_REPO}" ]; then \
		git push ${HPP_REPO}/$(@:.push=) HEAD:master; \
		git push ${HPP_REPO}/$(@:.push=) HEAD:devel; \
	fi


# Remove tag corresponding to the new package version
%.remove-tag:
	cd ${SRC_DIR}/$(@:.remove-tag=); \
	if [ "${$(@:.remove-tag=)_repository}" = "${HPP_REPO}" ]; then \
		git tag -d v${$(@:.remove-tag=)_version}; \
	fi

# if repository is HPP_REPO, print version of package 
%.print-version:
	@cd ${SRC_DIR}/$(@:.print-version=); \
	if [ "${$(@:.print-version=)_repository}" = "${HPP_REPO}" ]; then \
		echo "$(@:.print-version=): `pkg-config --modversion $(@:.print-version=)`"; \
	fi

# Push tag corresponding to the new package version to HPP_REPO
%.push-tag:
	cd ${SRC_DIR}/$(@:.push-tag=); \
	if [ "${$(@:.push-tag=)_version}" != "" ]; then \
		git push ${HPP_REPO}/$(@:.push-tag=) v${$(@:.push-tag=)_version}; \
	fi

# if variable is defined, call make release in package
# if tarball already present, nothing is done.
%.release:
	if [ "${$(@:.release=)_version}" != "" ]; then \
		echo "Releasing version ${$(@:.release=)_version} of package $(@:.release=)"; \
		if [ -f ${SRC_DIR}/$(@:.release=)/${BUILD_FOLDER}/$(@:.release=).${$(@:.release=)_version}.tar.gz ]; then \
			echo "$(@:.release=).${$(@:.release=)_version}.tar.gz already exists."; \
		else \
			cd ${SRC_DIR}/$(@:.release=)/${BUILD_FOLDER}; \
			make release VERSION=${$(@:.release=)_version}; \
			make dist; \
			scp ${SRC_DIR}/$(@:.release=)/${BUILD_FOLDER}/$(@:.release=)-${$(@:.release=)_version}.tar.gz www.openrobots.org:/var/www/html/distfiles/$(@:.release=)/.; \
		fi \
	fi
# for pinocchio, push in hpp branch.
pinocchio.push:
	@cd ${SRC_DIR}/$(@:.push=); \
	git push ${$(@:.push=)_repository}/$(@:.push=) HEAD:hpp

