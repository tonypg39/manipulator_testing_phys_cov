# Do the loop around 1,2,3

from gen_task import generate
from run_simulation import main_run, main_kill

# FIXCONFIG: Add connection for the max iterations
max_iter = 10
it = 0

while it < max_iter:
    generate()
    main_run()
    main_kill()

