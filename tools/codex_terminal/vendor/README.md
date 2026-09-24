# Bundled terminal dependencies

Pinned official npm packages, MIT licenses included alongside each asset:

| Package | Version | Files | Source tarball SHA-256 |
| --- | --- | --- | --- |
| `@xterm/xterm` | 6.0.0 | `xterm.js`, `xterm.css`, `xterm.LICENSE` | `908e66e04af6c8dc6b00dd3b54de088e2e81e5ed866284fd6c2fb3c2d1c7a3f6` |
| `@xterm/addon-fit` | 0.11.0 | `addon-fit.js`, `addon-fit.LICENSE` | `26003b4517a132b64e4ff228fd88a5fda3fff5e606c76093f6dcff772e9ecec0` |

Obtained with `npm pack @xterm/xterm@6.0.0 @xterm/addon-fit@0.11.0 --ignore-scripts`.
Only `lib/xterm.js`, `css/xterm.css`, `lib/addon-fit.js` and each package's `LICENSE` were copied. Runtime scripts are served locally; there are no CDN requests or package installation steps on MSI.

Upstream: <https://github.com/xtermjs/xterm.js>, <https://www.npmjs.com/package/@xterm/xterm>, <https://www.npmjs.com/package/@xterm/addon-fit>.
