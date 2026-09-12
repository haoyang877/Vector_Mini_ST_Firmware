"""Small, synchronous workflows for one motor-drive node."""

from collections.abc import Callable
from pathlib import Path
import time

from mdrive_core.protocol.flash_record import FlashRecordInfo, read_info, read_record
from mdrive_core.protocol.loader_client import LoaderClient, PhaseCallback, UpdateResult, update_firmware
from mdrive_core.protocol.param import Client, encode, raw_get
from mdrive_core.schema.params import PARAMS, READABLE_IDS
from mdrive_core.transport.base import CanTransport

Progress = Callable[[int, int], None]


class DeviceSessionError(ValueError):
    """A requested operation is not valid for the node session or schema."""


class DeviceSession:
    """Bind a transport channel and application node into one serial session."""

    def __init__(self, transport: CanTransport, node: int = 0, channel: int = 0, timeout: float = 0.08) -> None:
        if not 0 <= node <= 7:
            raise DeviceSessionError("node must be 0..7")
        self.transport = transport
        self.node = node
        self.channel = channel
        self.timeout = timeout
        self._parameters = Client(transport, timeout=timeout)

    def node_revision(self) -> int:
        """Return protocol revision 2 or reject an incompatible application."""
        return int(self._parameters.require_revision(self.channel, self.node))

    def read(self, set_id: int) -> float:
        """Read a writable parameter through its paired GET ID."""
        definition = PARAMS.get(set_id)
        if definition is None or definition.access != "rw":
            raise DeviceSessionError(f"parameter {set_id:#x} is not a writable SET ID")
        value, _ = self._parameters.read(self.channel, self.node, set_id)
        return value

    def read_raw(self, get_id: int) -> float:
        """Read a schema-defined GET ID directly."""
        definition = PARAMS.get(get_id)
        if definition is None or definition.access != "ro":
            raise DeviceSessionError(f"parameter {get_id:#x} is not a read-only GET ID")
        return raw_get(self.transport, self.channel, self.node, get_id, definition.encoding, self.timeout)

    def write(self, set_id: int, value: float) -> None:
        """Issue one non-retried parameter write."""
        definition = PARAMS.get(set_id)
        if definition is None or definition.access != "rw":
            raise DeviceSessionError(f"parameter {set_id:#x} is not writable")
        self._parameters.write(self.channel, self.node, set_id, value)

    def read_all(self, progress: Progress | None = None) -> dict[int, float | None]:
        """Read every schema-readable ID serially, recording failures as None."""
        readable = sorted(READABLE_IDS)
        values: dict[int, float | None] = {}
        for index, identifier in enumerate(readable, 1):
            definition = PARAMS[identifier]
            try:
                values[identifier] = self.read(identifier) if definition.access == "rw" else self.read_raw(identifier)
            except (TimeoutError, ValueError):
                values[identifier] = None
            if progress is not None:
                progress(index, len(readable))
        return values

    def save_params(self) -> None:
        """Request firmware persistence and observe its save-mode transition."""
        identifier = (self.node << 8) | 0x68
        self.transport.send(self.channel, identifier, encode(0x68, 1.0))
        deadline = time.perf_counter() + 3.0
        saw_save_mode = False
        while time.perf_counter() < deadline:
            try:
                mode = raw_get(self.transport, self.channel, self.node, 0x01, "f32", min(0.1, deadline - time.perf_counter()))
            except TimeoutError:
                continue
            if mode != 0:
                saw_save_mode = True
            elif saw_save_mode:
                return
        raise TimeoutError("parameter save timed out; firmware 0x68 not supported yet?")

    def flash_info(self) -> FlashRecordInfo:
        """Read the flash parameter record header (0x69 FLASH_PARAM_INFO)."""
        return read_info(self.transport, self.channel, self.node)

    def flash_read(self, progress: Progress | None = None) -> bytes:
        """Read and CRC-verify the whole flash parameter record (0x69 + 0x6A)."""
        info = read_info(self.transport, self.channel, self.node)
        return read_record(self.transport, self.channel, self.node, info=info, progress=progress)

    @staticmethod
    def scan(transport: CanTransport, channel: int = 0) -> dict[int, int]:
        """Probe application nodes 0 through 7 for protocol revision 2."""
        nodes: dict[int, int] = {}
        for node in range(8):
            try:
                nodes[node] = int(raw_get(transport, channel, node, 0x67, "f32", 0.08))
            except TimeoutError:
                continue
        return nodes

    def upgrade(
        self,
        image_path: Path,
        version: int,
        progress: Progress | None = None,
        on_phase: PhaseCallback | None = None,
        abort_event=None,
    ) -> UpdateResult:
        """Enter the resident Loader and update an image from disk."""
        image = image_path.read_bytes()
        client = LoaderClient(self.transport, node=self.node, channel=self.channel)
        client.enter_boot()
        client.wait_loader_alive()
        return update_firmware(client, image, version, progress, on_phase=on_phase, abort_event=abort_event)
