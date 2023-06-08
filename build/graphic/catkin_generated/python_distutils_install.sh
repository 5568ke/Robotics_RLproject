#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/src/graphic"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/install/lib/python3/dist-packages:/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/src/graphic/setup.py" \
     \
    build --build-base "/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/build/graphic" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/install" --install-scripts="/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/install/bin"
