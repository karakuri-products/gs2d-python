# ! /usr/bin/env python3
# encoding: utf-8

from abc import ABCMeta, abstractmethod


class ISerialInterface(metaclass=ABCMeta):
    @abstractmethod
    def write(self, data):
        raise NotImplementedError()

    @abstractmethod
    def readline(self):
        raise NotImplementedError()

    @abstractmethod
    def is_open(self):
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        raise NotImplementedError()
