# Examples

## Owl Symbolic
In this example, we build a model with [Owl Symbolic](https://github.com/owlbarn/owl_symbolic) and execute it with [OCaml ONNX Runtime](https://github.com/tachukao/ocaml-onnxruntime).

```sh
cd ocaml-onnxruntime
dune exec examples/symbolic.exe
```
To run this example, you will need to install [Owl Symbolic](https://github.com/owlbarn/).

## FNS Candy
We perform neural style transfer on an image `amber.jpg` ([link](https://github.com/gnsmrky/pytorch-fast-neural-style-for-web/blob/master/images/content-images/amber.jpg)), by executing a pretrained ONNX model `candy.onnx` ([link](https://github.com/onnx/models/tree/master/vision/style_transfer/fast_neural_style)).
To run this example, download `amber.jpg` and `candy.onnx` to a folder called `results` and run

```sh
cd ocaml-onnxruntime
dune exec ./examples/candy.exe -- -d path/to/results
```

This example requires:
1. [camlimages](https://opam.ocaml.org/packages/camlimages/)
2. [cmdargs](https://github.com/hennequin-lab/cmdargs)

