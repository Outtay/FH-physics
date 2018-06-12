def FlagsForFile( filename, **kwargs ):
  return {
    #'flags': [ '-x', 'c++', '-std=c++11','-Wall', '-Wextra', '-Werror', '-isystem', '/home/xubuntu/Documents/FH/git/reactphysics3d/src', 'isystem', '/home/otti/Documents/git/github/reactphysics3d/src' ],
    'flags': [ '-x', 'c++', '-std=c++11','-Wall', '-Wextra', '-Werror', '-isystem', '/home/xubuntu/Documents/FH/git/reactphysics3d/src', '-isystem', '../reactphysics3d/src' ],
  }
