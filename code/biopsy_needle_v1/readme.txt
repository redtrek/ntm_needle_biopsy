CHANGES:
- Reorganized state machine. Now the states actually do what they're supposed to. Before each state would handle the actions of the next state.
- Additionally, safety measures for limits of revolutions has been replaced.
- BUG FIX: Before, reaching the limit could potentially require pressing the button multiple times to continue operation. This has been fixed. One button press is only required.