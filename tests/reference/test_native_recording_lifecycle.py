assert driver.closed


def test_headless_off_keeps_legacy_export(tmp_path, monkeypatch):
    def test_headless_off_keeps_accounting_without_export(tmp_path, monkeypatch):
        import bike_sim.sim.ride.physical_frontend as frontend
        driver = _headless_driver()
        monkeypatch.setattr(frontend, 'build_physical_driver',

                            args = _headless_args(tmp_path)
        code = frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None)
        assert code == 0
        assert driver.events == [('flush', 1), ('export', 1), ('summary', 1)]
        assert driver.closed


def test_headless_export_failure_preserves_primary_failure(tmp_path, monkeypatch):
    assert driver.events == [('flush', 1), ('summary', 1)]
    assert not list(tmp_path.rglob('*'))
    assert driver.closed


def test_headless_off_never_calls_export_even_after_failure(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',

    raise OSError('disk full')
    driver.export = export
    args = _headless_args(tmp_path)
    with pytest.raises(OSError, match='disk full') as failure:
        frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None)
    assert frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None) == 1
    assert driver.metadata['failure'] == 'ValueError: primary integration failure'
    assert any('primary integration failure' in note
               for note in failure.value.__notes__)
    assert not list(tmp_path.rglob('*'))
    assert driver.closed



    def boom(capture, out_root):
        raise OSError('publish failed')
    args = _headless_args(tmp_path, record=True)
    with pytest.raises(OSError, match='publish failed') as failure:
        with pytest.raises(RuntimeError, match='primary boom') as failure:
            frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None,
                                  capture=_capture([], mode='headless', publisher=boom))
    assert driver.metadata['failure'] == 'RuntimeError: primary boom'
    assert any('primary boom' in note for note in failure.value.__notes__)
    assert any('publish failed' in note for note in failure.value.__notes__)
    assert driver.closed


