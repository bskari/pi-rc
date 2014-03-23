# vim: set syntax=python
debug_flags = '-g'
warning_flags = '-Wall -Wextra -Weffc++ -Wshadow -Wswitch-enum -Wswitch-default'
environment = Environment()

environment.Append(CXXFLAGS=' '.join((debug_flags, warning_flags)))

pi_fm_target = environment.Program(
    target='pi_fm',
    source=(
        'pi_fm.cpp',
        'pi_radio.cpp',
    )
)
pi_pcm_target = environment.Program(
    target='pi_pcm',
    source=(
        'pi_pcm.cpp',
        'pi_radio.cpp',
    )
)

Default(pi_pcm_target)
