#!/bin/bash
#http://wiki.oz9aec.net/index.php/Building_Gstreamer_1.2_from_source

NO_ERROR=0
BASE_PATH=/home/pi/gst

if [ ! -f dependency_install ]; then
   apt-get install libx264-dev libgudev-1.0-dev yasm bison flex
   touch dependency_install
fi

source ./set_env.sh
ORC_DOWNLOAD=http://gstreamer.freedesktop.org/src/orc/orc-0.4.21.tar.xz
FILES=( http://gstreamer.freedesktop.org/src/gstreamer/gstreamer-1.4.0.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-base/gst-plugins-base-1.4.0.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-good/gst-plugins-good-1.4.0.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-bad/gst-plugins-bad-1.4.0.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-plugins-ugly/gst-plugins-ugly-1.4.0.tar.xz \
        http://gstreamer.freedesktop.org/src/gst-libav/gst-libav-1.4.0.tar.xz )

if [ ! -f downloaded ]; then
   [ $NO_ERROR -eq 0 ] && wget -c $ORC_DOWNLOAD || NO_ERROR=1
   for file in ${FILES[@]}; do
      [ $NO_ERROR -eq 0 ] && wget -c $file || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && wget -c $file.sha256sum || NO_ERROR=1
   done
   [ $NO_ERROR -eq 0 ] && sha256sum -c *.sha256sum || NO_ERROR=1
   [ $NO_ERROR -eq 0 ] && touch downloaded
fi

for file in $(ls -1 | grep -E -e "gz$|xz$"); do
   #echo "$(echo "$file" | sed 's/\.tar\.[gx]z$//')"
   if [ ! -d $(echo "$file" | sed 's/\.tar\.[gx]z$//') ]; then
      echo "Extracting $file"
      [ $NO_ERROR -eq 0 ] && tar xf $file || NO_ERROR=1
   fi
done

PREFIX="--prefix=/home/pi/gst/runtime"
ORC="--enable-orc"

pushd $BASE_PATH/orc-*
   CURRENT=$BASE_PATH/orc_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gstreamer-*
   CURRENT=$BASE_PATH/gstreamer_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gst-plugins-base-*
   CURRENT=$BASE_PATH/gst-plugins-base_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gst-plugins-good-*
   CURRENT=$BASE_PATH/gst-plugins-good_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gst-plugins-bad-*
   CURRENT=$BASE_PATH/gst-plugins-bad_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gst-plugins-ugly-*
   CURRENT=$BASE_PATH/gst-plugins-ugly_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd

pushd $BASE_PATH/gst-libav-*
   CURRENT=$BASE_PATH/gst-libav_status
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "configured") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && ./configure $ORC $PREFIX || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "configured" > $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "built") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "built" >> $CURRENT
   fi
   if [ ! -f $CURRENT ] || [ $(cat $CURRENT | grep -c "installed") -eq 0 ]; then
      [ $NO_ERROR -eq 0 ] && make install || NO_ERROR=1
      [ $NO_ERROR -eq 0 ] && echo "installed" >> $CURRENT
   fi
popd
