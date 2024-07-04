from __future__ import annotations


class Observable:
    def __init__(self):
        self.observers: list[Observer] = []

    def add_observer(self, observer: Observer) -> None:
        if observer not in self.observers:
            self.observers.append(observer)

    def remove_observer(self, observer: Observer) -> None:
        if observer in self.observers:
            self.observers.remove(observer)

    def notify_observers(self, *args, **kwargs) -> None:
        for observer in self.observers:
            observer.update(self, *args, **kwargs)


class Observer:
    def update(self, observable: Observable, *args, **kwargs):
        pass
