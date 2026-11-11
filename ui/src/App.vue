<!--
  Chamber Controller UI
  Copyright (c) 2021-2026 Magnus

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
-->

<template>
  <dialog id="spinner" class="loading">
    <div class="container text-center">
      <div class="row align-items-center" style="height: 170px">
        <div class="col">
          <div class="spinner-border" role="status" style="width: 5rem; height: 5rem">
            <span class="visually-hidden">Loading...</span>
          </div>
        </div>
      </div>
    </div>
  </dialog>

  <div v-if="!global.initialized" class="container text-center">
    <BsMessage
      message="Initializing Chamber Controller Web interface"
      :dismissable="false"
      alert="info"
    ></BsMessage>
  </div>

  <BsMenuBar
    v-if="global.initialized"
    :disabled="global.disabled"
    brand="Chamber Controller"
    :menu-items="items"
    :dark-mode="config.dark_mode"
    :mdns="config.mdns"
    :config-changed="global.configChanged"
    @update:dark-mode="handleDarkModeUpdate"
  />

  <div class="container">
    <div>
      <p></p>
    </div>
    <BsMessage
      v-if="!status.connected"
      message="No response from device, has it gone into sleep mode? No need to refresh the page, just turn on the device again"
      :dismissable="false"
      alert="danger"
    ></BsMessage>

    <BsMessage
      v-if="global.isError"
      :close="close"
      :dismissable="true"
      :message="global.messageError"
      alert="danger"
    />
    <BsMessage
      v-if="global.isWarning"
      :close="close"
      :dismissable="true"
      :message="global.messageWarning"
      alert="warning"
    />
    <BsMessage
      v-if="global.isSuccess"
      :close="close"
      :dismissable="true"
      :message="global.messageSuccess"
      alert="success"
    />
    <BsMessage
      v-if="global.isInfo"
      :close="close"
      :dismissable="true"
      :message="global.messageInfo"
      alert="info"
    />

    <BsMessage v-if="status.wifi_setup" :dismissable="false" alert="info">
      Running in WIFI setup mode. Go to the
      <router-link class="alert-link" to="/device/wifi">wifi settings</router-link> meny and select
      wifi. Restart device after settings are selected.
    </BsMessage>
  </div>

  <router-view v-if="global.initialized" />
  <BsFooter
    v-if="global.initialized"
    :text="
      '(c) 2024-2026 Magnus Persson, ui version ' + global.uiVersion + ' (' + global.uiBuild + ')'
    "
  />
</template>

<script setup>
import { watch, onBeforeMount, onBeforeUnmount, ref } from 'vue'
import { global, status, config, saveConfigState } from '@/modules/pinia'
import { sharedHttpClient as http } from '@mp-se/espframework-ui-components'
import { items as staticItems } from '@/modules/router'
import { computed } from 'vue'
import { logError } from '@mp-se/espframework-ui-components'

const items = computed(() => JSON.parse(JSON.stringify(staticItems.value)))

const polling = ref(null)
const close = (alert) => {
  if (alert == 'danger') global.messageError = ''
  else if (alert == 'warning') global.messageWarning = ''
  else if (alert == 'success') global.messageSuccess = ''
  else if (alert == 'info') global.messageInfo = ''
}

watch(
  () => global.disabled,
  () => {
    if (global.disabled) document.body.style.cursor = 'wait'
    else document.body.style.cursor = 'default'
  }
)

function ping() {
  ;(async () => {
    const ok = await http.ping()
    status.connected = ok
  })()
}

function showSpinner() {
  const spinner = document.querySelector('#spinner')
  if (spinner) spinner.showModal()
}

function hideSpinner() {
  const spinner = document.querySelector('#spinner')
  if (spinner) spinner.close()
}

async function initializeApp() {
  try {
    showSpinner()

    // Step 1: Authenticate with device (http client owns token)
    const base = btoa('chamber:password')
    const authOk = await http.auth(base)
    if (!authOk) {
      global.messageError = 'Failed to authenticate with device, please try to reload page!'
      return
    }

    // Step 2: Load feature flags
    const globalSuccess = await global.load()
    if (!globalSuccess) {
      global.messageError = 'Failed to load feature flags from device, please try to reload page!'
      return
    }

    // Step 3: Load device status
    const statusSuccess = await status.load()
    if (!statusSuccess) {
      global.messageError = 'Failed to load status from device, please try to reload page!'
      return
    }

    // Step 4: Load configuration
    const configSuccess = await config.load()
    if (!configSuccess) {
      global.messageError =
        'Failed to load configuration data from device, please try to reload page!'
      return
    }

    // Success! Initialize the app
    saveConfigState()
    global.initialized = true
  } catch (error) {
    logError('App.initializeApp()', error)
    global.messageError = `Initialization failed: ${error.message}`
  } finally {
    hideSpinner()
  }
}

// Handle dark mode changes
const handleDarkModeUpdate = (newValue) => {
  // update the store value
  config.dark_mode = newValue
  // fallback: ensure the attribute is set on the document root so Bootstrap theme rules apply
  try {
    const root = document.documentElement
    if (newValue) root.setAttribute('data-bs-theme', 'dark')
    else root.setAttribute('data-bs-theme', 'light')
  } catch (e) {
    console.error('Failed to set data-bs-theme on documentElement', e)
  }
}

// Watch for changes to config.dark_mode and call handleDarkModeUpdate
watch(
  () => config.dark_mode,
  (newValue) => {
    handleDarkModeUpdate(newValue)
  }
)

onBeforeMount(() => {
  initializeApp()
  polling.value = setInterval(ping, 7000)
})

onBeforeUnmount(() => {
  if (polling.value) {
    clearInterval(polling.value)
  }
})
</script>

<style>
.loading {
  position: fixed;
  width: 200px;
  height: 200px;
  padding: 10px;
  top: 0;
  right: 0;
  bottom: 0;
  left: 0;
  border: 0;
}

dialog::backdrop {
  background-color: black;
  opacity: 60%;
}
</style>
