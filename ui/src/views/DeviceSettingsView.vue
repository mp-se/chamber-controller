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
  <div class="container">
    <p></p>
    <p class="h3">{{ t('device_settings.title') }}</p>
    <hr />

    <BsMessage v-if="config.mdns === ''" dismissable="true" message="" alert="warning">
      {{ t('device_settings.mdns_warning') }}
    </BsMessage>

    <form @submit.prevent="saveSettings" class="needs-validation" novalidate>
      <div class="row">
        <div class="col-md-12">
          <BsInputText
            v-model="config.mdns"
            type="text"
            :label="t('device_settings.mdns_label')"
            :help="t('device_settings.mdns_help')"
            required
            :disabled="global.disabled"
          ></BsInputText>
        </div>

        <div class="col-md-12">
          <hr />
        </div>

        <div class="col-md-4">
          <BsInputRadio
            v-model="config.temp_format"
            :options="tempOptions"
            :label="t('device_settings.temp_format_label')"
            :disabled="global.disabled"
          ></BsInputRadio>
        </div>

        <div class="col-md-4">
          <BsInputRadio
            v-model="config.dark_mode"
            :options="uiOptions"
            :label="t('device_settings.ui_label')"
            :disabled="global.disabled"
          ></BsInputRadio>
        </div>

        <div class="col-md-4">
          <BsSelect
            v-model="config.restart_interval"
            :options="restartOptions"
            :label="t('device_settings.restart_interval_label')"
            :help="t('device_settings.restart_interval_help')"
            :disabled="global.disabled"
          ></BsSelect>
        </div>
      </div>

      <div class="row gy-2">
        <div class="col-md-12">
          <hr />
        </div>
        <div class="col-md-12">
          <button
            type="submit"
            class="btn btn-primary w-2"
            :disabled="global.disabled || !global.configChanged"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;{{ t('device_settings.save') }}</button
          >&nbsp;

          <button
            @click.prevent="restartDevice"
            type="button"
            class="btn btn-secondary"
            :disabled="global.disabled"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;{{ t('device_settings.restart') }}</button
          >&nbsp;

          <button
            @click.prevent="factory"
            type="button"
            class="btn btn-secondary"
            :disabled="global.disabled"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;{{ t('device_settings.factory_defaults') }}</button
          >&nbsp;
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { validateCurrentForm, logDebug } from '@mp-se/espframework-ui-components'
import { useI18n } from 'vue-i18n'
import { ref, onMounted } from 'vue'
import { global, config } from '@/modules/pinia'

const { t } = useI18n()

const tempOptions = ref([
  { label: t('device_settings.temp_celsius'), value: 'C' },
  { label: t('device_settings.temp_fahrenheit'), value: 'F' }
])

const uiOptions = ref([
  { label: t('device_settings.ui_day_mode'), value: false },
  { label: t('device_settings.ui_dark_mode'), value: true }
])

const restartOptions = ref([
  { label: t('device_settings.restart_disabled'), value: 0 },
  { label: t('device_settings.restart_30m'), value: 30 },
  { label: t('device_settings.restart_1h'), value: 60 },
  { label: t('device_settings.restart_2h'), value: 120 },
  { label: t('device_settings.restart_4h'), value: 240 },
  { label: t('device_settings.restart_6h'), value: 360 },
  { label: t('device_settings.restart_12h'), value: 720 },
  { label: t('device_settings.restart_24h'), value: 1440 }
])

onMounted(() => {
  logDebug('DeviceSettingsView.onMounted()')
})

const saveSettings = async () => {
  if (!validateCurrentForm()) return
  try {
    await config.saveAll()
    global.messageSuccess = t('device_settings.save_success') || 'Settings saved'
  } catch {
    global.messageError = t('device_settings.err_save_failed')
  }
}

const restartDevice = async () => {
  try {
    await config.restart()
  } catch {
    global.messageError = t('device_settings.err_restart_failed')
  }
}

const factory = async () => {
  try {
    global.clearMessages()
    global.disabled = true

    const response = await fetch(global.baseURL + 'api/factory', {
      headers: { Authorization: global.token },
      signal: AbortSignal.timeout(global.fetchTimeout)
    })

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`)
    }

    const json = await response.json()

    if (json.success === true) {
      global.messageSuccess = t('messages.FACTORY_RESET_COMPLETED') + ' Reloading page in 2 seconds...'

      const reloadTimeout = setTimeout(() => {
        try {
          location.reload(true)
        } catch (error) {
          logDebug('DeviceSettingsView.factory.reload() error: ' + error)
          window.location.reload()
        }
      }, 2000)

      window.addEventListener(
        'beforeunload',
        () => {
          clearTimeout(reloadTimeout)
        },
        { once: true }
      )
    } else {
      global.messageError = json.message || 'Factory restore failed'
    }
  } catch (err) {
    logDebug('DeviceSettingsView.factory() error: ' + err)
    global.messageError = 'Failed to perform factory restore: ' + (err.message || err)
  } finally {
    global.disabled = false
  }
}

defineExpose({
  tempOptions,
  uiOptions,
  restartOptions,
  saveSettings,
  restartDevice,
  factory
})
</script>
