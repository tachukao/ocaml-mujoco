# OCaml Bindings to MuJoCo

## Installation

1. Install MuJoCo 210 to the default directory `~/.mujoco/mjoco210` from [here](https://mujoco.org/download)

2. Install GLFW3 

3. Install GLFW3 OCaml bindings
```sh
opam install glfw-ocaml
```

4. Install Mujoco bindings
```sh
dune build @install
dune install
```

## Example
```sh
dune exec ./examples/basic.exe -- -xml model/rodent.xml 
dune exec ./examples/basic.exe -- -xml model/humanoid.xml 
```
