from abc import ABC, abstractmethod
from typing import Iterable

from dl_control.interface.entity.torch import State


class ObsNNInterface(ABC):
    @abstractmethod
    def prepare_state(self, state: State) -> Iterable[float]:
        pass
