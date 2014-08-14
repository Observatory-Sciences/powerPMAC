#!/bin/sh

# Install libssh2 under /usr/local/lib
LIBSSH2_VER=1.4.3
export LIBSSH2_VER


if ! test -e /usr/local/lib/libssh2.a;
then
  echo "libssh2 not found in /usr/local/lib"
  echo "Do you want to install libssh2 in /usr/local/lib ?"
  echo "[y/N]"
  read yn
  if test "$yn" != y; then
    echo "you didn't press y, so nothing is done"
    exit 1  
  fi 
  #apt or yum
  if type apt-get 2>/dev/null; then
    APTGET="sudo apt-get install"
  else
    if type yum 2>/dev/null; then
      APTGET="sudo /usr/bin/yum install"
    fi
  fi
  export APTGET
  #
  #openssl
  (
    test -r /usr/include/ssl.h || test -r /usr/include/openssl/ssl.h || {
      echo $APTGET openssl &&
      $APTGET openssl
    }
  ) &&
  # libgcrypt-dev
  if ! test -r /usr/include/gcrypt/h && ! test -r /opt/local/include/gcrypt.h; then
    (
      echo $APTGET libgcrypt-dev &&
      $APTGET libgcrypt-dev 
    ) ||
    (
      echo $APTGET libgcrypt-devel &&
      $APTGET libgcrypt-devel 
    ) 
  fi &&
  #
  if ! type gcc 2>/dev/null; then
    echo $APTGET lbuild-essential &&
    $APTGET lbuild-essential
  fi &&
  if ! test -r libssh2-$LIBSSH2_VER.tar.gz; then
    wget http://www.libssh2.org/download/libssh2-$LIBSSH2_VER.tar.gz -O libssh2-${LIBSSH2_VER}.tar.gz.$$.tmp &&
    mv libssh2-${LIBSSH2_VER}.tar.gz.$$.tmp libssh2-${LIBSSH2_VER}.tar.gz || {
      echo >&2 can not wget libssh2-${LIBSSH2_VER}.tar.gz
      exit 1
    }
  fi &&
  if ! test -d libssh2-$LIBSSH2_VER; then
    gunzip libssh2-$LIBSSH2_VER.tar.gz &&
    tar -xvf libssh2-$LIBSSH2_VER.tar || {
      echo >&2 can not untar ungz libssh2-${LIBSSH2_VER}.tar.gz
      exit 1
    }
  fi &&
  (
    cd libssh2-$LIBSSH2_VER &&
    ./configure &&
    make &&
    sudo make install
  ) &&
  echo libssh2.a is now installed
else
  echo libssh2.a is already installed
fi
