const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const CopyWebpackPlugin = require("copy-webpack-plugin");

module.exports = {
  entry: "./src/index.jsx",
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "bundle.js",
    clean: true
  },
  resolve: {
    extensions: [".js", ".jsx"]
  },
  module: {
    rules: [
      {
        test: /\.jsx?$/,
        exclude: /node_modules/,
        use: "babel-loader"
      }
    ]
  },
  plugins: [
    new HtmlWebpackPlugin({ template: "./public/index.html" }),
    // planner.wasm/.data are fetched at runtime by planner.js; copy them as-is.
    new CopyWebpackPlugin({
      patterns: [{ from: "public/wasm", to: "wasm", noErrorOnMissing: true }]
    })
  ],
  devServer: {
    static: { directory: path.resolve(__dirname, "public") },
    port: 5173,
    host: "0.0.0.0"
  }
};
