# Quarto and LaTeX rendering

## Brackets at the start of an aligned block

Use `\left[\boldsymbol\omega_s\right]_\times` instead of `[\boldsymbol\omega_s]_\times` at the start of an `aligned` block. A leading `[` can be interpreted as the environment's optional alignment argument.

```latex
\begin{aligned}
\left[\boldsymbol\omega_s\right]_\times
&=\dot{\mathbf R}_{se}\mathbf R_{se}^{\mathsf T}
\end{aligned}
```
