{
  inputs = {
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixpkgs.url = "nixpkgs/nixos-unstable";
  };

  outputs = {
    self,
    fenix,
    nixpkgs,
  }: let
    # Define supported systems
    supportedSystems = ["x86_64-linux" "aarch64-linux" "aarch64-darwin"];

    # Helper function to generate outputs for each system
    forAllSystems = nixpkgs.lib.genAttrs supportedSystems;

    # Define the Rust toolchain components
    rustToolchainComponents = [
      "cargo"
      "clippy"
      "rust-src"
      "rustc"
      "rustfmt"
      "rust-analyzer"
    ];
  in {
    # Development shells for each supported system
    devShells = forAllSystems (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      rustToolchain = fenix.packages.${system}.combine [
        (fenix.packages.${system}.complete.withComponents rustToolchainComponents)
        fenix.packages.${system}.targets.wasm32-unknown-unknown.latest.rust-std
      ];
    in {
      default = pkgs.mkShell {
        name = "rust-dev-shell";
        packages = [
          rustToolchain
          pkgs.uv
          pkgs.python3
          pkgs.wasm-pack
          pkgs.nodejs
          pkgs.typescript-language-server
          pkgs.eslint
          pkgs.just
          pkgs.mdbook
        ];

        shellHook = ''
          VENV_DIR=".venv"

          if [ ! -d "$VENV_DIR" ]; then
            echo "Creating Python virtual environment at $VENV_DIR..."
            uv venv $VENV_DIR -p ${pkgs.python3}/bin/python
          fi

          source "$VENV_DIR/bin/activate"

          uv pip install --quiet -r requirements.txt

          uv pip install pre-commit
          pre-commit install
          # zsh
        '';
      };
    });
  };
}
