include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node default {

  # We need build tools to compile
  class {'build_tools': }

  # These user tools make the shell much easier
  class {'user_tools':
    user => 'vagrant',
  }

  # Get and install our toolchain
  $toolchain_version = '2.1.1'
  wget::fetch { 'nubots_deb':
    destination => "/root/nubots-toolchain-${toolchain_version}.deb",
    source      => "http://nubots.net/debs/nubots-toolchain-${toolchain_version}.deb",
    timeout     => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure   => 'latest',
    source   => "/root/nubots-toolchain-${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
  $archs = {
    'native'    => {'flags'       => ['-m64', ],
                    'params'      => ['', ],
                    'environment' => {'TARGET' => 'GENERIC', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                   },
    'fitpc2i'   => {'flags'       => ['-m32', '-march=bonnell', '-mtune=bonnell', '-mno-movbe', '-mfxsr', '-mmmx', '-msahf', '-msse', '-msse2', '-msse3', '-mssse3', ],
                    'params'      => ['--param l1-cache-size=24', '--param l1-cache-line-size=64', '--param l2-cache-size=512', ],
                    'environment' => {'TARGET' => 'YONAH', 'USE_THREAD' => '1', 'BINARY' => '32', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m32', },
                   },
    'nuc7i7bnh' => {'flags'       => ['-m64', '-march=broadwell', '-mtune=broadwell', '-mmmx', '-mno-3dnow', '-msse', '-msse2', '-msse3', '-mssse3', '-mno-sse4a', '-mcx16', '-msahf', '-mmovbe', '-maes', '-mno-sha', '-mpclmul', '-mpopcnt', '-mabm', '-mno-lwp', '-mfma', '-mno-fma4', '-mno-xop', '-mbmi', '-mbmi2', '-mno-tbm', '-mavx', '-mavx2', '-msse4.2', '-msse4.1', '-mlzcnt', '-mno-rtm', '-mno-hle', '-mrdrnd', '-mf16c', '-mfsgsbase', '-mrdseed', '-mprfchw', '-madx', '-mfxsr', '-mxsave', '-mxsaveopt', '-mno-avx512f', '-mno-avx512er', '-mno-avx512cd', '-mno-avx512pf', '-mno-prefetchwt1', '-mclflushopt', '-mxsavec', '-mxsaves', '-mno-avx512dq', '-mno-avx512bw', '-mno-avx512vl', '-mno-avx512ifma', '-mno-avx512vbmi', '-mno-clwb', '-mno-mwaitx', ],
                    'params'      => ['--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                    'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                   },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need build tools to compile and we need it done before the installer
  class {'build_tools': } -> Installer <| |>

  # These user tools make the shell much easier and these also should be done before installing
  class {'user_tools':
    user => 'vagrant',
  } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'nuc7i7bnh' => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'prebuild'    => 'make distclean',
                       'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                       'method'      => 'autotools', },
    'zlib'         => {'url'         => 'http://www.zlib.net/zlib-1.2.11.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake',},
    'bzip2'        => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.1.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'fitpc2i' => [ '', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'creates'     => 'lib/libbz2.so',
                       'method'      => 'make',},
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'nuc7i7bnh' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools',},
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/release/1.0.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'fitpc2i' => [ '-DBUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake',},
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.19.tar.gz',
                       'method'      => 'make',
                       'creates'     => 'lib/libopenblas.a' },
    'libsvm'       => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                       'creates'     => 'lib/svm.o',
                       'method'      => 'make', },
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.950.1.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ],},
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5.93/gperftools-2.5.93.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'nuc7i7bnh' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools',},
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'fitpc2i' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'nuc7i7bnh' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake',},
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.6-pl2.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-fortran', '--enable-shared', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', ], },
                       'method'      => 'autotools',},
    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
                       'args'        => { 'native'   => [ 'CCASFLAGS="-f elf64"', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', 'CCASFLAGS="-f elf32"', ],
                                          'nuc7i7bnh' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools',},
    'cppformat'    => {'url'         => 'https://github.com/fmtlib/fmt/archive/3.0.1.tar.gz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libfmt.a' },
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'args'        => { 'native'   => [ '', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'method'      => 'autotools',},
    'muparserx'    => {'url'         => 'https://github.com/beltoforion/muparserx/archive/v4.0.7.tar.gz',
                       'method'      => 'cmake',},
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',},
    'boost'        => {'url'         => 'https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz',
                       'args'        => { 'native'   => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'fitpc2i' => [ 'address-model=32', 'architecture=x86', 'link=static', ],
                                          'nuc7i7bnh' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2'], ],},
    'espeak'       => {'url'         => 'https://github.com/Bidski/espeak/archive/v1.48.04.tar.gz',
                       'src_dir'     => 'src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ Installer['portaudio'], ],},
    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/releases/download/1.9.3/fswatch-1.9.3.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'fitpc2i' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'method'      => 'autotools', },
  }

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Hash,
                          Optional['require'] => Tuple[Any, 1, default],
                          'method' => String,
                          Optional['src_dir'] => String,
                          Optional['prebuild'] => String,
                          Optional['postbuild'] => String}] $params| {

        $extension = $params['url'] ? {
          /.*\.zip/       => 'zip',
          /.*\.tgz/       => 'tgz',
          /.*\.tar\.gz/   => 'tar.gz',
          /.*\.txz/       => 'txz',
          /.*\.tar\.xz/   => 'tar.xz',
          /.*\.tbz/       => 'tbz',
          /.*\.tbz2/      => 'tbz2',
          /.*\.tar\.bz2/  => 'tar.bz2',
          /.*\.h/         => 'h',
          /.*\.hpp/       => 'hpp',
          default         => 'UNKNOWN',
        }

        archive { "${archive}":
          url              => $params['url'],
          target           => "/nubots/toolchain/src/${archive}",
          src_target       => "/nubots/toolchain/src",
          purge_target     => true,
          checksum         => false,
          follow_redirects => true,
          timeout          => 0,
          extension        => $extension,
          strip_components => 1,
          root_dir         => '.',
          require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['build_tools'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          extension   => $extension,
        }
  }

  # Install quex.
  class { 'quex': }

  # Install protobuf.
  class { 'protobuf': }

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
    method    => 'wget',
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    # Update the armadillo config header file for all archs.
    file { "armadillo_${arch}_config":
      path    => "/nubots/toolchain/${arch}/include/armadillo_bits/config.hpp",
      source  => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
      ensure  => present,
      require => [ Installer['armadillo'], ],
    }
  }

  archive { "Spinnaker_nuc7i7bnh":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_amd64.tar.gz",
    target           => "/nubots/toolchain/nuc7i7bnh/src/Spinnaker",
    src_target       => "/nubots/toolchain/nuc7i7bnh/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
  }
  archive { "Spinnaker_native":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_amd64.tar.gz",
    target           => "/nubots/toolchain/native/src/Spinnaker",
    src_target       => "/nubots/toolchain/native/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
  }
  archive { "Spinnaker_fitpc2i":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_i386.tar.gz",
    target           => "/nubots/toolchain/fitpc2i/src/Spinnaker",
    src_target       => "/nubots/toolchain/fitpc2i/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
  }
  exec { "Spinnaker_nuc7i7bnh":
    creates  => "/nubots/toolchain/nuc7i7bnh/include/Spinnaker.h",
    command  => "cd include && cp -r ./* /nubots/toolchain/nuc7i7bnh/include/ && cd .. &&
                 cd lib &&
                 cp libGCBase_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libGenApi_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libLog_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libMathParser_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libNodeMapData_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libptgreyvideoencoder.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libSpinnaker.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cp libXmlParser_gcc540_v3_0.so* /nubots/toolchain/nuc7i7bnh/lib/ &&
                 cd ..",
    cwd      => "/nubots/toolchain/nuc7i7bnh/src/Spinnaker",
    path     =>  [ "/nubots/toolchain/nuc7i7bnh/bin", "/nubots/toolchain/bin",
                   '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout  => 0,
    provider => 'shell',
    require  => [ Archive["Spinnaker_nuc7i7bnh"], ],
    before   => Class['toolchain_deb'],
  }
  exec { "Spinnaker_native":
    creates  => "/nubots/toolchain/native/include/Spinnaker.h",
    command  => "cd include && cp -r ./* /nubots/toolchain/native/include/ && cd .. &&
                 cd lib &&
                 cp libGCBase_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cp libGenApi_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cp libLog_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cp libMathParser_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cp libNodeMapData_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cp libptgreyvideoencoder.so* /nubots/toolchain/native/lib/ &&
                 cp libSpinnaker.so* /nubots/toolchain/native/lib/ &&
                 cp libXmlParser_gcc540_v3_0.so* /nubots/toolchain/native/lib/ &&
                 cd ..",
    cwd      => "/nubots/toolchain/native/src/Spinnaker",
    path     =>  [ "/nubots/toolchain/native/bin", "/nubots/toolchain/bin",
                   '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout  => 0,
    provider => 'shell',
    require  => [ Archive["Spinnaker_native"], ],
    before   => Class['toolchain_deb'],
  }
  exec { "Spinnaker_fitpc2i":
    creates  => "/nubots/toolchain/fitpc2i/include/Spinnaker.h",
    command  => "cd include && cp -r ./* /nubots/toolchain/fitpc2i/include/ && cd .. &&
                 cd lib &&
                 cp libGCBase_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libGenApi_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libLog_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libMathParser_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libNodeMapData_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libptgreyvideoencoder.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libSpinnaker.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cp libXmlParser_gcc540_v3_0.so* /nubots/toolchain/fitpc2i/lib/ &&
                 cd ..",
    cwd      => "/nubots/toolchain/fitpc2i/src/Spinnaker",
    path     =>  [ "/nubots/toolchain/fitpc2i/bin", "/nubots/toolchain/bin",
                   '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout  => 0,
    provider => 'shell',
    require  => [ Archive["Spinnaker_fitpc2i"], ],
    before   => Class['toolchain_deb'],
  }

  # After we have installed, create the CMake toolchain files and then build our deb.
  Installer <| |> ~> class { 'toolchain_deb': }

  $archs.each |String $arch, Hash $params| {
    # Create CMake toolchain files.
    $prefix          = '/nubots/toolchain'
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\")
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\")

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }
  }
}
