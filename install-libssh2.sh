#!/bin/sh

# Install libssh2 under /usr/local/lib
LIBSSH2_VER=1.4.3
export LIBSSH2_VER

wget_or_curl()
{
  url=$1
  file=$2
  if type wget >/dev/null; then
    wget "$url" -O "$file.$$.tmp" &&
    mv "$file.$$.tmp" "$file" || {
      echo >&2 wget can not get $url
      exit 1
    }
  else
    curl "$url" >"$file.$$.tmp" &&
    mv "$file.$$.tmp" "$file" || {
      echo >&2 curl can not get $url
      exit 1
    }
  fi
}

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
  APTGET=/bin/false
  uname_s=$(uname -s)
  case $uname_s in
    Darwin)
    if type port 2>/dev/null; then
      APTGET="sudo port install"
    else
      echo >&2 "It seams as if you run Mac OS X/Darwin"
      echo >&2 "You need to install mac ports to run this script"
      echo >&2 "https://www.macports.org/"
      exit 1
    fi
    ;;
    Linux)
    if type apt-get 2>/dev/null; then
      APTGET="sudo apt-get install"
    else
      if type yum 2>/dev/null; then
        APTGET="sudo /usr/bin/yum install"
      fi
    fi
    ;;
    *)
    echo >&2 "Unsupported system :$uname_s"
    exit 1
  esac
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
    ) ||
    (
      echo $APTGET libgcrypt &&
      $APTGET libgcrypt 
    ) 
  fi &&
  #
  if ! type gcc 2>/dev/null; then
    echo $APTGET lbuild-essential &&
    $APTGET lbuild-essential
  fi &&
  if ! test -r libssh2-$LIBSSH2_VER.tar.gz; then
    wget_or_curl http://www.libssh2.org/download/libssh2-$LIBSSH2_VER.tar.gz libssh2-${LIBSSH2_VER}.tar.gz
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
