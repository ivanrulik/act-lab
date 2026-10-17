"""Pinned optional numeric-action projection; preserve current RGB observations."""

from collections.abc import Iterator
from contextlib import contextmanager
from importlib import import_module
from typing import Any

import torch  # type: ignore[import-not-found]
from lerobot.datasets.dataset_reader import (  # type: ignore[import-not-found]
    DatasetReader,
)


class ProjectedActionReader(DatasetReader):  # type: ignore[misc]
    """Keep upstream indexing/masks, and format only numeric future actions."""

    def _query_hf_dataset(self, query_indices: dict[str, list[int]]) -> dict[str, Any]:
        others = {
            key: values for key, values in query_indices.items() if key != "action"
        }
        result: dict[str, Any] = super()._query_hf_dataset(others)
        if "action" in query_indices:
            indices = query_indices["action"]
            if self._absolute_to_relative_idx is not None:
                indices = [self._absolute_to_relative_idx[i] for i in indices]
            values = self.hf_dataset.select_columns(["action"])[indices]["action"]
            result["action"] = torch.stack(values)
        return result


def project_action_reader(dataset: Any) -> Any:
    """Adapt the inspected class in place, preserving its complete reader state."""
    if type(dataset.reader) is not DatasetReader:
        raise RuntimeError("action projection requires the pinned DatasetReader")
    dataset.reader.__class__ = ProjectedActionReader
    return dataset


@contextmanager
def projected_action_factory() -> Iterator[None]:
    """Limit the factory hook to one synchronous training call in this process."""
    module: Any = import_module("lerobot.scripts.lerobot_train")
    original = module.make_train_eval_datasets

    def make_datasets(*args: Any, **kwargs: Any) -> Any:
        train, evaluation = original(*args, **kwargs)
        return (
            project_action_reader(train),
            project_action_reader(evaluation) if evaluation is not None else None,
        )

    module.make_train_eval_datasets = make_datasets
    try:
        yield
    finally:
        module.make_train_eval_datasets = original
