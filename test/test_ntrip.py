

def test_email_caster_falls_back_to_an_email(monkeypatch):
    from mhseals_nav.rtk import ntrip
    caster = {'name': 'RTK2go', 'user_env': 'RTK2GO_USER', 'user_default': 'email'}
    monkeypatch.setenv('EMAIL', 'crew@example.com')
    assert ntrip.credentials(caster, {})[0] == 'crew@example.com'
    assert ntrip.credentials(caster, {'RTK2GO_USER': 'set@example.com'})[0] == 'set@example.com'
    assert ntrip.credentials({'name': 'X', 'user_env': 'X_USER'}, {})[0] is None   # no default: stays unset
