"""Checks which execution provider open_runner settles on, GPUs first and then the CPU."""

from types import SimpleNamespace

import pytest

from sub_vision import backends

CUDA = "CUDAExecutionProvider"
MIGRAPHX = "MIGraphXExecutionProvider"
CPU = backends.CPU_PROVIDER
SHAPE = (1, 3, 64, 64)


def fake_onnxruntime(monkeypatch, available, no_start=(), no_kernels=()):
    """ONNX Runtime with these providers: ``no_start`` silently leave the session on the CPU, as
    a provider without its GPU does, and ``no_kernels`` fail their first inference, as one without
    kernels for this GPU does."""

    class Session:
        def __init__(self, _path, _options, providers, enable_fallback):
            assert enable_fallback == 0
            self.provider = CPU if providers[0] in no_start else providers[0]

        def get_providers(self):
            return [self.provider, CPU]

        def get_modelmeta(self):
            return SimpleNamespace(custom_metadata_map={})

        def get_inputs(self):
            return [SimpleNamespace(name="images", shape=list(SHAPE))]

        def run(self, _outputs, _feed):
            if self.provider in no_kernels:
                raise RuntimeError("no kernel image is available for execution on the device")
            return []

    monkeypatch.setattr(backends.ort, "get_available_providers", lambda: list(available))
    monkeypatch.setattr(backends.ort, "InferenceSession", Session)


@pytest.fixture
def model(tmp_path):
    path = tmp_path / "model.onnx"
    path.touch()
    return str(path)


def open_on(device, model):
    logged = []
    return backends.open_runner(model, device, SHAPE, logged.append), logged


@pytest.mark.parametrize("device", ["auto", "gpu"])
def test_the_first_gpu_provider_in_preference_order_runs_it(monkeypatch, model, device):
    # conda-forge's x86 CUDA build lists TensorRT first, without TensorRT installed.
    fake_onnxruntime(monkeypatch, ["TensorrtExecutionProvider", MIGRAPHX, CUDA, CPU])
    runner, logged = open_on(device, model)
    assert runner.name == "onnxruntime/cuda"
    assert logged == []


@pytest.mark.parametrize(
    "broken", [{"no_start": [CUDA]}, {"no_kernels": [CUDA]}], ids=["no_start", "no_kernels"]
)
def test_a_gpu_that_cannot_run_it_gives_way_to_the_next(monkeypatch, model, broken):
    fake_onnxruntime(monkeypatch, [CUDA, MIGRAPHX, CPU], **broken)
    runner, _ = open_on("gpu", model)
    assert runner.name == "onnxruntime/migraphx"


@pytest.mark.parametrize(
    "broken", [{"no_start": [CUDA]}, {"no_kernels": [CUDA]}], ids=["no_start", "no_kernels"]
)
def test_auto_falls_back_to_the_cpu_and_says_why(monkeypatch, model, broken):
    fake_onnxruntime(monkeypatch, [CUDA, CPU], **broken)
    runner, logged = open_on("auto", model)
    assert runner.name == "onnxruntime/cpu"
    assert len(logged) == 1 and CUDA in logged[0]


def test_auto_without_a_gpu_provider_uses_the_cpu_quietly(monkeypatch, model):
    fake_onnxruntime(monkeypatch, [CPU])
    runner, logged = open_on("auto", model)
    assert runner.name == "onnxruntime/cpu"
    assert logged == []


@pytest.mark.parametrize(
    ("available", "broken", "reason"),
    [
        ([CUDA, CPU], {"no_kernels": [CUDA]}, "no kernel image"),
        ([CUDA, CPU], {"no_start": [CUDA]}, "did not start"),
        ([CPU], {}, "only has"),
    ],
    ids=["no_kernels", "no_start", "no_provider"],
)
def test_gpu_refuses_the_cpu_and_says_why(monkeypatch, model, available, broken, reason):
    fake_onnxruntime(monkeypatch, available, **broken)
    with pytest.raises(RuntimeError, match=reason):
        open_on("gpu", model)


def test_cpu_never_tries_a_gpu(monkeypatch, model):
    fake_onnxruntime(monkeypatch, [CUDA, CPU], no_kernels=[CUDA])
    runner, logged = open_on("cpu", model)
    assert runner.name == "onnxruntime/cpu"
    assert logged == []


def test_a_missing_model_is_an_error(monkeypatch, tmp_path):
    fake_onnxruntime(monkeypatch, [CPU])
    with pytest.raises(FileNotFoundError):
        open_on("auto", str(tmp_path / "missing.onnx"))
