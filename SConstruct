# vim: set syntax=python
debug_flags = '-g'
warning_flags = '-Wall -Wextra -Wshadow -Wswitch-enum' \
    ' -Wswitch-default'
c_warning_flags = '-Wmissing-prototypes -Wmissing-declarations' \
    ' -Wstrict-prototypes'
cxx_warning_flags = '-Weffc++'
other_compiler_flags = '-Werror'
linker_flags = '-lm -ljansson'

environment = Environment()

environment.Append(
    CFLAGS=' '.join((
        debug_flags,
        warning_flags,
        c_warning_flags,
        other_compiler_flags,
    ))
)
environment.Append(
    CXXFLAGS=' '.join((
        debug_flags,
        warning_flags,
        cxx_warning_flags,
        other_compiler_flags,
    ))
)

environment.Append(LINKFLAGS=linker_flags)

pi_pcm_target = environment.Program(
    target='pi_pcm',
    source=(
        'pi_pcm.c',
    )
)

Default(pi_pcm_target)
