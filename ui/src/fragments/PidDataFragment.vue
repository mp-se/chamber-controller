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
  <p class="h4">{{ dataSource }}</p>

  <div class="row">
    <pre>{{ dataFormatted }}</pre>
  </div>
  <div class="row">
    <div class="col">
      <button @click="load()" type="button" class="btn btn-primary w-4" :disabled="global.disabled">
        <span
          class="spinner-border spinner-border-sm"
          role="status"
          aria-hidden="true"
          v-show="global.disabled"
        ></span>
        &nbsp;{{ t('pid.reload') }}
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, computed } from 'vue'
import { global } from '@/modules/pinia'
import { logError, logDebug, sharedHttpClient as http } from '@mp-se/espframework-ui-components'
import { useI18n } from 'vue-i18n'

const { t } = useI18n()

const data = ref('')
const source = defineModel('source')

const dataSource = computed(() => {
  if (source.value == 'cc') return t('pid.control_constants')
  if (source.value == 'cv') return t('pid.control_variables')
  if (source.value == 'mt') return t('pid.min_times')
  if (source.value == 'cs') return t('pid.control_settings')
  return t('pid.unknown')
})

const dataFormatted = computed(() => {
  try {
    //    return JSON.stringify(JSON.parse(data.value), null, 2)
    return JSON.stringify(data.value, null, 2)
  } catch (e) {
    logError('PidDataFragment::dataFormatted()', e)
  }
  return t('pid.fetching')
})

onMounted(() => {
  load()
})

const load = async () => {
  logDebug('DevicePidsView:load()')
  global.clearMessages()
  global.disabled = true
  try {
    const json = await http.getJson('api/pid/' + source.value)
    logDebug('DevicePidsView:load()', json)
    data.value = json
    global.disabled = false
  } catch (err) {
    logError('DevicePidsView:load()', err)
    global.messageError = t('pid.err_load', { source: '/api/pid/' + source.value })
    global.disabled = false
  }
}
</script>
