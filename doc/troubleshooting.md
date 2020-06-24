
# Troubleshooting

## Processes hanging

### clearing the shared memory

A recurring issue when using o80 is that processes may hang if trying to connect to a deprecated shared memory. A deprecated shared memory occurs when a process is not cleanly exited (i.e. the destructor methods are not properly called).

This can be solved by called the o80::clear_shared_memory method using the suitable segment_id as argument.

o80::clear_shared_memory has not down side, and it is even good manner to call this function before a new standalone is instantiated.

```python

segment_id = "id0"
o80.clear_shared_memory(segment_id)
o80_robot.start_standalone(segment_id, ...)

```

If the clear_shared_memory does not work, you may try to delete the content of the folder /dev/shm. Note that this may also delete shared memory files used by process not related to o80.

### instantiating in the right order

A FrontEnd targeting a segment_id must always be started *after* a corresponding standalone or BackEnd has been instantiated. If the standalone or BackEnd is destroyed before the FrontEnd, the instance of FrontEnd will hang.




