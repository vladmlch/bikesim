# Offline plot resources

The native reporter always emits a self-contained HTML file. Its built-in,
first-party `native-html-v1` SVG renderer is embedded by `viz/html.cpp`; it
supports pan, zoom, trace toggles, unified hover, missing points, secondary
axes, annotations, data export and SVG export without an external file.

The HTML metadata names that renderer accurately. It does not claim to be
Plotly. When a licensed upstream `plotly.min.js` is installed in this directory,
or supplied through `viz::set_plotly_resource`, generation embeds the actual
bundle and uses Plotly. The resulting HTML still needs no network connection.

The uploaded source snapshot and this editing environment contain no Plotly
bundle. No third-party bundle, license or digest was fabricated. Acquisition
and locking of the upstream bundle and its MIT license remains a release
dependency verification gate. First-party SVG/HTML and PNG generation do not
depend on that optional resource being available.
