"""Compile the versioned wire schema for wheels and editable installations."""

from pathlib import Path

import grpc_tools
from grpc_tools import protoc
from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        target = Path(self.root) / "build" / "act_lab_episode.desc"
        foxglove_target = Path(self.root) / "build" / "act_lab_foxglove.desc"
        protobuf_include = Path(grpc_tools.__file__).parent / "_proto"
        target.parent.mkdir(parents=True, exist_ok=True)
        result = protoc.main(
            [
                "protoc",
                f"-I{self.root}/proto",
                f"-I{protobuf_include}",
                "--include_imports",
                f"--descriptor_set_out={target}",
                f"{self.root}/proto/act_lab/v1/episode.proto",
            ]
        )
        if result:
            raise RuntimeError("episode Protobuf compilation failed")
        build_data["force_include"][str(target)] = "act_lab_episode.desc"
        result = protoc.main(
            [
                "protoc",
                f"-I{self.root}/proto",
                f"-I{protobuf_include}",
                "--include_imports",
                f"--descriptor_set_out={foxglove_target}",
                f"{self.root}/proto/foxglove/RawImage.proto",
            ]
        )
        if result:
            raise RuntimeError("Foxglove Protobuf compilation failed")
        build_data["force_include"][str(foxglove_target)] = "act_lab_foxglove.desc"
