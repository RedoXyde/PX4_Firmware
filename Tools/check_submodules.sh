#!/usr/bin/env bash

[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
    # GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules
    echo "Skipping submodules. NUTTX_SRC is set to $NUTTX_SRC"
    exit 0
}

GITSTATUS=$(git status)

function check_git_submodule {

if [ -d $1 ];
	then
	SUBMODULE_STATUS=$(git submodule summary "$1")
	STATUSRETVAL=$(echo $SUBMODULE_STATUS | grep -A20 -i "$1" | grep "<")
	if [ -z "$STATUSRETVAL" ]; then
		echo "Checked $1 submodule, correct version found"
	else
		echo -e "\033[31mChecked $1 submodule, ACTION REQUIRED:"
		echo ""
		echo ""
		echo -e "New commits required:"
		echo -e "$SUBMODULE_STATUS\033[0m"
		echo ""
		echo ""
		echo "   $1 submodule not at correct version. Did you intentionally change the submodule?"
		echo "   If yes, hit 'y' and <ENTER> to continue the build. If not, hit <ENTER> to abort."
		echo -e "   Hit 'u' and <ENTER> to update ALL submodules and resolve this (performs \033[94mgit submodule update --init --recursive\033[0m)."
		echo -e "   Use \033[94mgit add $1 && git commit -m 'Updated $1'\033[0m to choose this submodule version (careful!)"
		echo ""
		read user_cmd
		if [ "$user_cmd" == "y" ]
		then
			echo "Continuing build with manually overridden submodule.."
		else
			if [ "$user_cmd" == "u" ]
			then
				git submodule update --init --recursive
				echo "Submodule fixed, continuing build.."
			else
				echo "Build aborted."
				exit 1
			fi
		fi
	fi
else
	git submodule update --init --recursive;
	git submodule update;
fi

}

check_git_submodule NuttX
check_git_submodule Tools/gencpp
check_git_submodule Tools/genmsg
check_git_submodule Tools/jMAVSim
check_git_submodule Tools/sitl_gazebo
check_git_submodule cmake/cmake_hexagon
check_git_submodule mavlink/include/mavlink/v1.0
check_git_submodule src/lib/DriverFramework
check_git_submodule src/lib/dspal
check_git_submodule src/lib/ecl
check_git_submodule src/lib/matrix
check_git_submodule src/modules/uavcan/libuavcan
check_git_submodule unittests/googletest

exit 0