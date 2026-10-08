"""Native ride-runtime boundary: capability validation, artifact loading and
the t=0 setup bootstrap for the owned C++ physical runtime (design section 3).

Nothing in this package imports the compiled extension eagerly — loading is
explicit through ``bike_sim.native.artifact.load_native_extension`` so
Python-only frontends stay free of the native dependency.
"""
