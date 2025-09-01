from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, Optional, TypeVar
from helper.logger import prepare_logger

# Variance: send type is contravariant (you can accept broader inputs),
# recv type is covariant (you can return narrower outputs).
SendT = TypeVar("SendT", contravariant=True)
RecvT = TypeVar("RecvT", covariant=True)


class Link(Generic[SendT, RecvT], ABC):
    """
    Abstract class to handle communications between Raspberry Pi and other components.

    Subclasses choose their payload types:
      - send(message: SendT) -> None
      - recv() -> Optional[RecvT]
    """

    def __init__(self) -> None:
        self.logger = prepare_logger()

    @abstractmethod
    def send(self, message: SendT) -> None:
        ...

    @abstractmethod
    def recv(self) -> Optional[RecvT]:
        ...
