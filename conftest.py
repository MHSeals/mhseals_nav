"""Root pytest config: the quarantine plugin of mhseals_agents, a no-op when it is not on the path."""

pytest_plugins = ['mhseals_agents.pytest_quarantine'] if __import__('importlib.util').util.find_spec('mhseals_agents') else []  # noqa: E501
