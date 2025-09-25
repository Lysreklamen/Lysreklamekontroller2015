{
  description = "Dev env for c (avr)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { self, nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      crossPkgs = pkgs.pkgsCross.avr;
    in
    {
      # Dev-shell with avr
      # Run `nix develop .` to activate the dev-shell
      devShells.${system} = {
        default = pkgs.mkShell {
          packages = [
            crossPkgs.buildPackages.gcc       # provides `avr-gcc`
            crossPkgs.buildPackages.binutils  # AVR binutils
            crossPkgs.libc                    # avr-libc
            pkgs.avrdude
            pkgs.cmake
            pkgs.gnumake
          ];

          shellHook = ''
            export NIX_SHELL_NAME="c"
          '';
        };
      };
    };
}
