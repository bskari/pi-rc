# vim: set syntax=python
debug_flags = '-g'
warning_flags = '-Wall -Wextra -Weffc++ -Wshadow -Wswitch-enum -Wswitch-default'
environment = Environment()

environment.Append(CXXFLAGS=' '.join((debug_flags, warning_flags)))

pi_radio_target = environment.Program(
    target='pi_radio',
    source=(
        'pi_radio.cpp',
    )
)

Default(pi_radio_target)
