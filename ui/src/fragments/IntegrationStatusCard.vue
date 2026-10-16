<template>
  <BsCard :header="t('home.integration')" :color="color" :title="name">
    <p class="text-center">{{ message }}</p>
  </BsCard>
</template>

<script setup>
import { computed } from 'vue'
import { useI18n } from 'vue-i18n'

const props = defineProps({
  name: { type: String, required: true },
  status: { type: Object, required: true }
})

const { t } = useI18n()

const age = computed(() => {
  const seconds = Math.floor((props.status.push_age || 0) / 1000)
  if (seconds < 60) return `${seconds}s`
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m`
  return `${Math.floor(seconds / 3600)}h`
})

const color = computed(() => {
  if (!props.status.push_used) return 'secondary'
  return props.status.push_status ? 'success' : 'danger'
})

const message = computed(() => {
  if (!props.status.push_used) return t('home.integration_not_updated')
  if (props.status.push_status) return t('home.integration_success', { age: age.value })
  return t('home.integration_failed', {
    age: age.value,
    code: props.status.push_code
  })
})
</script>
