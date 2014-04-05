# vim: set syntax=python
debug_flags = '-g'
c_warning_flags = '-Wall -Wextra -Wshadow -Wswitch-enum -Wswitch-default'
cxx_warning_flags = c_warning_flags + ' -Weffc++'
linker_flags = '-lm'
environment = Environment()

environment.Append(
    CFLAGS=' '.join((debug_flags, c_warning_flags)),
    CXXFLAGS=' '.join((debug_flags, cxx_warning_flags)),
    LINKFLAGS=linker_flags,
)

pi_fm_target = environment.Program(
    target='pi_fm',
    source=(
        'pi_fm.c',
        'pi_radio.c',
    )
)

Default(pi_fm_target)
