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
  <div class="card h-100" v-if="status.remote_control_active === false">
    <div class="card-header bg-success-subtle">{{ t('pid.controller_title') }}</div>
    <div class="card-body">
      <form @submit.prevent="saveSettings" class="needs-validation" novalidate>
        <div class="row">
          <div class="col-md-8">
            <div class="row text-start">
              <BsInputRadio
                v-model="newMode"
                :label="t('pid.change_mode')"
                :options="modeOptions"
                :disabled="global.disabled"
              />
            </div>
            <div class="row text-start">
              <BsInputNumber
                v-model="newTemperature"
                :label="t('pid.target_temp')"
                min="0"
                max="30"
                step="0.1"
                :unit="'°' + config.temp_format"
                width="6"
                :disabled="global.disabled || modeOptions.length == 1"
              />
            </div>
          </div>
          <div class="col-md-4 text-center align-self-center justify-content-center">
            <div class="row">
              <button
                type="submit"
                class="btn btn-primary"
                style="height: 100px; width: 100px"
                :disabled="global.disabled || modeOptions.length == 1"
              >
                {{ t('pid.set') }}
              </button>
            </div>
          </div>
        </div>
      </form>
    </div>
  </div>

  <div class="card h-100" v-else>
    <div class="card-header bg-success-subtle">{{ t('pid.controller_title') }}</div>
    <div class="card-body">
      <p class="text-center">{{ t('pid.remote_active') }}</p>
      <button
        @click="disableRemoteControl"
        type="button"
        class="btn btn-primary"
        :disabled="global.disabled"
      >
        {{ t('pid.disable_remote') }}
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import {
  logDebug,
  logError,
  logInfo,
  validateCurrentForm,
  sharedHttpClient as http
} from '@mp-se/espframework-ui-components'
import { global, config, status } from '@/modules/pinia'
import { storeToRefs } from 'pinia'
import { useI18n } from 'vue-i18n'

const { t } = useI18n()

const newMode = ref('o')
const newTemperature = ref(config.target_temperature)

const modeOptions = ref([{ label: t('pid.off'), value: 'o' }])

const { pid_mode } = storeToRefs(status)

watch(pid_mode, () => {
  newMode.value = pid_mode.value
})

onMounted(() => {
  // We need at least one sensor to do control
  if (config.enable_cooling || config.enable_heating) {
    if (config.beer_sensor_id.length || config.fridge_sensor_id.length) {
      if (config.beer_sensor_id.length)
        modeOptions.value.push({ label: t('home.beer_constant'), value: 'b' })

      if (config.fridge_sensor_id.length)
        modeOptions.value.push({ label: t('home.fridge_constant'), value: 'f' })
    } else {
      global.messageError = t('pid.err_no_sensors')
    }
  } else {
    global.messageError = t('pid.err_no_capabilities')
  }
})

const disableRemoteControl = async () => {
  global.clearMessages()
  global.disabled = true

  try {
    const data = { new_mode: 'r' }
    await http.postJson('api/remote', data)
    global.disabled = false
    logInfo('PidControllerFragment.disableRemoteControl()', 'Sending /api/remote completed')
    return true
  } catch (err) {
    logError('PidControllerFragment.disableRemoteControl()', err)
    global.disabled = false
    return false
  }
}

const saveSettings = async () => {
  try {
    if (!validateCurrentForm()) return

    global.disabled = true
    global.clearMessages()

    const data = {
      new_mode: newMode.value,
      new_temperature: newTemperature.value
    }

    logDebug('PidControllerFragment.saveSettings()', data)
    logInfo('PidControllerFragment.saveSettings()', 'Sending /api/mode')

    await http.postJson('api/mode', data)

    logInfo('PidControllerFragment.saveSettings()', 'Sending /api/mode completed')
    global.messageSuccess = t('pid.save_success')
  } catch (err) {
    logError('PidControllerFragment.saveSettings()', err)
    global.messageError = t('pid.err_save_failed', { error: err.message || err })
  } finally {
    global.disabled = false
  }
}
</script>
