/*
 * OrcaRouter provider settings UI.
 *
 * The browser is a view only: it never holds an API key beyond the value the
 * user is typing into the masked input, and every catalog/inference call is
 * made server-side.  The stored credential is only ever shown masked.
 */
(function () {
  "use strict";

  var api = {
    status: "/api/status",
    models: "/api/models",
    saveKey: "/api/credentials/api-key",
    clear: "/api/credentials/clear",
    connectStart: "/api/connect/start",
    connectPoll: "/api/connect/poll",
    connectComplete: "/api/connect/complete",
    connectCancel: "/api/connect/cancel",
  };

  var state = {
    // Monotonic login generation.  Every async response must still belong to
    // the current generation before it is allowed to change anything.
    generation: 0,
    // Finished response of the attempt currently rendered.
    generationAtStart: 0,
    busy: false,
    flow: "A",
    pollTimer: null,
    // Incremented per catalog request so a slow response for an older surface
    // can never overwrite the options for the surface now selected.
    catalogToken: 0,
    // The surface key the currently rendered options belong to.  Until a
    // refresh resolves this is the *previous* surface, so consumers must not
    // treat the option list as belonging to the newly selected surface yet.
    loadedKey: null,
    surface: "chat",
    modalities: [],
    models: [],
    catalogDegraded: false,
    selectedModel: null,
  };

  function $(id) { return document.getElementById(id); }

  function post(url, body) {
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
      credentials: "omit",
    }).then(function (res) {
      return res.json().catch(function () { return { ok: false, error: "bad response" }; });
    });
  }

  function get(url) {
    return fetch(url, { credentials: "omit" }).then(function (res) {
      return res.json().catch(function () { return { ok: false, error: "bad response" }; });
    });
  }

  function setStatus(el, text, kind) {
    el.textContent = text;
    el.classList.remove("is-ok", "is-error");
    if (kind === "ok") { el.classList.add("is-ok"); }
    if (kind === "error") { el.classList.add("is-error"); }
  }

  function describe(account) {
    if (!account) { return "not configured"; }
    if (account.state === "ready") {
      return "connected " + account.masked_key +
        (account.account_id ? " (account " + account.account_id + ")" : "");
    }
    if (account.state === "needs_reauth") {
      return "needs reauthentication: " + account.detail;
    }
    return "not configured";
  }

  // --- status ---------------------------------------------------------

  // The status endpoint returns provider entries (with the credential source
  // each one drives) and the account statuses keyed by that same source.  A
  // view joins them rather than assuming the two id spaces coincide.
  function elementIdFor(sourceId, providers) {
    var match = (providers || []).filter(function (p) {
      return p.credential_source === sourceId;
    })[0];
    return match ? "status-" + match.id : null;
  }

  function refreshStatus() {
    return get(api.status).then(function (data) {
      if (data.origins) {
        $("orca-catalog-url").textContent = data.origins.api_base + "/models";
      }
      var accounts = data.accounts || [];
      accounts.forEach(function (account) {
        var id = elementIdFor(account.source_id, data.providers);
        var el = id ? $(id) : null;
        if (el) {
          setStatus(el, describe(account),
            account.state === "needs_reauth" ? "error" :
            account.state === "ready" ? "ok" : "");
        }
      });
    });
  }

  // --- API-key adapter ------------------------------------------------

  $("orca-save-key").addEventListener("click", function () {
    var input = $("orca-api-key");
    var value = input.value;
    if (!value) {
      setStatus($("status-orcarouter"), "enter an sk-orca-… key first", "error");
      return;
    }
    post(api.saveKey, { api_key: value }).then(function (res) {
      // The key is never echoed back; clear the field as soon as it is stored.
      input.value = "";
      if (!res.ok) {
        setStatus($("status-orcarouter"), res.error || "could not save key", "error");
        return;
      }
      setStatus($("status-orcarouter"), describe(res.status), "ok");
      refreshCatalog();
    });
  });

  $("orca-clear-key").addEventListener("click", function () {
    post(api.clear, { source_id: "orcarouter" }).then(function () {
      $("orca-api-key").value = "";
      refreshStatus();
      refreshCatalog();
    });
  });

  // --- PKCE adapter ---------------------------------------------------

  function clearPollTimer() {
    if (state.pollTimer) {
      window.clearTimeout(state.pollTimer);
      state.pollTimer = null;
    }
  }

  function setBusy(busy, hint) {
    state.busy = busy;
    if (!busy) { state.generationAtStart = 0; }
    $("orca-connect").disabled = busy;
    $("orca-cancel-connect").disabled = !busy;
    var hintEl = $("orca-connect-hint");
    if (busy && hint) {
      hintEl.hidden = false;
      hintEl.textContent = hint;
    } else if (!busy) {
      hintEl.hidden = true;
      hintEl.textContent = "";
    }
  }

  function pollLogin(generation) {
    if (generation !== state.generation) { return; }
    state.pollTimer = window.setTimeout(function () {
      get(api.connectPoll + "?generation=" + encodeURIComponent(generation))
        .then(function (res) {
          // Drop anything belonging to a superseded generation.
          if (generation !== state.generation) { return; }
          if (res.state === "pending") { pollLogin(generation); return; }
          clearPollTimer();
          setBusy(false);
          if (res.state === "done") {
            setStatus($("status-orcarouter-oauth"), describe(res.status), "ok");
            refreshCatalog();
            return;
          }
          if (res.state === "superseded") { return; }
          setStatus($("status-orcarouter-oauth"), res.error || "login failed", "error");
        });
    }, 700);
  }

  $("orca-connect").addEventListener("click", function () {
    setStatus($("status-orcarouter-oauth"), "starting…", "");
    post(api.connectStart, { flow: "A" }).then(function (res) {
      if (!res.ok) {
        setStatus($("status-orcarouter-oauth"),
          res.busy ? "a login is already in progress" : (res.error || "could not start"),
          "error");
        return;
      }
      state.generation = res.generation;
      // The generation the page is currently rendering; the pagehide beacon
      // must carry the attempt it actually owns, not a stale default.
      state.generationAtStart = res.generation;
      state.flow = res.flow;
      setBusy(true, "Waiting for your browser to approve this app…");
      $("orca-oob-row").hidden = res.flow !== "B";
      pollLogin(res.generation);
    });
  });

  $("orca-oob-submit").addEventListener("click", function () {
    var code = $("orca-oob-code").value;
    post(api.connectComplete, { generation: state.generation, code: code }).then(function (res) {
      setBusy(false);
      if (!res.ok) {
        setStatus($("status-orcarouter-oauth"), res.error || "exchange failed", "error");
        return;
      }
      $("orca-oob-code").value = "";
      setStatus($("status-orcarouter-oauth"), describe(res.status), "ok");
      refreshCatalog();
    });
  });

  function cancelLogin() {
    clearPollTimer();
    // Clear the visible busy state synchronously: the server-side lock is
    // released by the same call.
    setBusy(false);
    return post(api.connectCancel, { generation: state.generation });
  }

  $("orca-cancel-connect").addEventListener("click", function () {
    cancelLogin().then(function () {
      setStatus($("status-orcarouter-oauth"), "login cancelled", "");
    });
  });

  $("orca-clear-oauth").addEventListener("click", function () {
    post(api.clear, { source_id: "orcarouter-oauth" }).then(function () {
      refreshStatus();
    });
  });

  // Switching provider or authentication method releases the lock too.
  $("orca-api-key").addEventListener("focus", function () {
    if (state.busy) { cancelLogin(); }
  });

  /*
   * pagehide can put this page into the back-forward cache.  The invalidated
   * worker's guarded cleanup deliberately refuses to mutate state, so busy and
   * hint must be cleared *here*, synchronously, before the page is frozen.
   * The server cancellation is then sent with keepalive so it survives.
   */
  window.addEventListener("pagehide", function () {
    if (!state.busy) { return; }
    // Read the attempt this page owns *before* clearing busy state, which
    // resets it: a beacon carrying generation 0 would not release the
    // server-side lock for the attempt that is actually in flight.
    var ownedGeneration = state.generationAtStart;
    clearPollTimer();
    state.generation += 1;      // invalidate any late response
    setBusy(false);             // synchronous: no permanent busy state on restore
    $("orca-oob-row").hidden = true;
    try {
      var payload = JSON.stringify({ generation: ownedGeneration });
      if (navigator.sendBeacon) {
        navigator.sendBeacon(api.connectCancel,
          new Blob([payload], { type: "application/json" }));
      } else {
        fetch(api.connectCancel, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: payload,
          keepalive: true,
          credentials: "omit",
        });
      }
    } catch (err) { /* best effort: never throw during unload */ }
  });

  // --- model selector -------------------------------------------------

  function surfaceSelection() {
    var value = $("orca-surface").value.split("|");
    return { capability: value[0], modalities: value[1] ? [value[1]] : [] };
  }

  function closeListbox() {
    $("orca-model-listbox").hidden = true;
    $("orca-model-trigger").setAttribute("aria-expanded", "false");
  }

  function openListbox() {
    if ($("orca-model-trigger").disabled) { return; }
    renderOptions();
    $("orca-model-listbox").hidden = false;
    $("orca-model-trigger").setAttribute("aria-expanded", "true");
  }

  function renderOptions() {
    var list = $("orca-model-listbox");
    list.innerHTML = "";
    if (!state.models.length) {
      var empty = document.createElement("div");
      empty.className = "orca-empty";
      empty.textContent = "No model matches this surface.";
      list.appendChild(empty);
      return;
    }
    state.models.forEach(function (model) {
      var option = document.createElement("div");
      option.className = "orca-option";
      option.setAttribute("role", "option");
      option.setAttribute("data-model-id", model.id);
      option.setAttribute("aria-selected",
        state.selectedModel === model.id ? "true" : "false");
      var label = document.createElement("span");
      label.textContent = model.name || model.id;
      var id = document.createElement("span");
      id.className = "orca-option-id";
      id.textContent = model.id;
      option.appendChild(label);
      option.appendChild(id);
      option.addEventListener("click", function () {
        selectModel(model);
        closeListbox();
      });
      list.appendChild(option);
    });
  }

  function selectModel(model) {
    state.selectedModel = model.id;
    $("orca-model-id").value = model.id;
    $("orca-model-value").textContent = model.name || model.id;
  }

  function clearSelection() {
    state.selectedModel = null;
    $("orca-model-id").value = "";
    $("orca-model-value").textContent = "Select a model…";
  }

  function surfaceKey(selection) {
    return selection.capability + "|" + selection.modalities.join(",");
  }

  function refreshCatalog() {
    var selection = surfaceSelection();
    var key = surfaceKey(selection);
    var token = ++state.catalogToken;
    var url = api.models + "?capability=" + encodeURIComponent(selection.capability) +
      "&modalities=" + encodeURIComponent(selection.modalities.join(","));
    $("orca-catalog-state").textContent = "loading catalog…";
    $("orca-model-trigger").disabled = true;
    state.loadedKey = null;
    return get(url).then(function (data) {
      if (token !== state.catalogToken) { return; }   // stale: newer request wins
      if (data.error === "needs_reauth") {
        clearSelection();
        closeListbox();
        state.models = [];
        state.loadedKey = key;
        $("orca-catalog-state").textContent = data.detail || "reconnect required";
        $("orca-catalog-state").classList.add("is-error");
        refreshStatus();
        return;
      }
      state.models = data.models || [];
      state.catalogDegraded = !!data.degraded;
      state.loadedKey = key;
      $("orca-catalog-state").classList.remove("is-error");
      $("orca-catalog-state").textContent =
        state.models.length + " models · source: " + data.source +
        (data.degraded ? " (degraded: " + (data.detail || "live discovery unavailable") + ")" : "");
      // A previously selected model that is no longer compatible is cleared
      // rather than silently kept.
      if (state.selectedModel &&
          !state.models.some(function (m) { return m.id === state.selectedModel; })) {
        clearSelection();
      }
      $("orca-model-trigger").disabled = state.models.length === 0;
      if (!$("orca-model-listbox").hidden) { renderOptions(); }
    });
  }

  $("orca-model-trigger").addEventListener("click", function () {
    if ($("orca-model-listbox").hidden) { openListbox(); } else { closeListbox(); }
  });

  $("orca-refresh-models").addEventListener("click", function () {
    refreshCatalog();
  });

  $("orca-surface").addEventListener("change", function () {
    var selection = surfaceSelection();
    state.surface = selection.capability;
    state.modalities = selection.modalities;
    closeListbox();
    // Changing surface can invalidate the current pick; refreshCatalog clears
    // it if it is no longer compatible.
    refreshCatalog();
  });

  document.addEventListener("click", function (event) {
    if (!$("orca-model-listbox").hidden &&
        !$("orca-model-listbox").contains(event.target) &&
        event.target !== $("orca-model-trigger")) {
      closeListbox();
    }
  });

  document.addEventListener("keydown", function (event) {
    if (event.key === "Escape") { closeListbox(); }
  });

  // Initial load.  The status is fetched first so the stored credential is on
  // screen before the (much slower) live catalog request completes.
  state.ready = false;
  refreshStatus().then(function () {
    state.ready = true;
    return refreshCatalog();
  });

  // Expose a tiny surface for the automated UI tests.
  window.OrcaRouterUI = {
    state: state,
    refreshCatalog: refreshCatalog,
    refreshStatus: refreshStatus,
    openListbox: openListbox,
    closeListbox: closeListbox,
    // The surface key the rendered options belong to, or null while a refresh
    // is in flight.  Consumers wait for it to match the selected surface.
    catalogReady: function () { return state.loadedKey; },
    surfaceKey: function () { return surfaceKey(surfaceSelection()); },
  };
})();
