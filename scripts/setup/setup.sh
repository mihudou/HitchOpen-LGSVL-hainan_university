
# setup main repo
git clone git@github.com:intelligentracing/HitchOpen-LGSVL.git
git checkout mwu/feat/planning_and_control # TODO: remove after merge
sudo apt update
sudo apt install python3-vcstool

cd HitchOpen-LGSVL
make vcs-import VCS_FILE=common.iron.repos
make vcs-import VCS_FILE=svl.iron.repos # if working on SVL simulator

make rosdep-install-eol
pip3 install environs

colcon build --packages-up-to simple_racing
source install/setup.bash
