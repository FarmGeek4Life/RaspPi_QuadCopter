#!/bin/bash

source ./set_env.sh
FILES=( http://gstreamer.freedesktop.org/src/orc/orc-0.4.18.tar.gz \
        http://gstreamer.freedesktop.org/src/gstreamer/gstreamer-1.2.3.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-base/gst-plugins-base-1.2.3.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-good/gst-plugins-good-1.2.3.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-bad/gst-plugins-bad-1.2.3.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-ugly/gst-plugins-ugly-1.2.3.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-libav/gst-libav-1.2.3.tar.xz )

for file in ${FILES[@]}; do
   wget -c $file
   wget -c $file.sha256sum
done

sha256sum -c *.sha256sum

for file in $(ls -1 |grep -E -e "gz$|xz$"); do
   if [ ! -d $(echo "$file" | grep -E -v -e "tar\.(gz$|xz$)") ]; then
      echo "Extracting $file"
      tar xf $file
   fi
done

PREFIX="--prefix=/home/pi/gst/runtime"
ORC="--enable-orc"

pushd orc*
   ./configure $PREFIX
   make install
popd

pushd gstreamer*
   ./configure $ORC $PREFIX
   make install
popd

pushd gst-plugins-base*
   ./configure $ORC $PREFIX
   make install
popd

pushd gst-plugins-good*
   ./configure $ORC $PREFIX
   make install
popd

pushd gst-plugins-bad*
   ./configure $ORC $PREFIX
   make install
popd

pushd gst-plugins-ugly*
   ./configure $ORC $PREFIX
   make install
popd

pushd gst-libav*
   ./configure $ORC $PREFIX
   make install
popd
