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

echo_and_run cd "/home/ros1/Documents/fyp2/src/ROS-TCP-Endpoint-main"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ros1/Documents/fyp2/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ros1/Documents/fyp2/install/lib/python3/dist-packages:/home/ros1/Documents/fyp2/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ros1/Documents/fyp2/build" \
    "/usr/bin/python3" \
    "/home/ros1/Documents/fyp2/src/ROS-TCP-Endpoint-main/setup.py" \
    egg_info --egg-base /home/ros1/Documents/fyp2/build/ROS-TCP-Endpoint-main \
    build --build-base "/home/ros1/Documents/fyp2/build/ROS-TCP-Endpoint-main" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ros1/Documents/fyp2/install" --install-scripts="/home/ros1/Documents/fyp2/install/bin"
