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

echo_and_run cd "/home/anaisjeger/catkin_ws/src/my_listener"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/anaisjeger/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/anaisjeger/catkin_ws/install/lib/python3/dist-packages:/home/anaisjeger/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/anaisjeger/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/anaisjeger/catkin_ws/src/my_listener/setup.py" \
     \
    build --build-base "/home/anaisjeger/catkin_ws/build/my_listener" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/anaisjeger/catkin_ws/install" --install-scripts="/home/anaisjeger/catkin_ws/install/bin"
