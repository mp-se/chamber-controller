import { describe, expect, it } from 'vitest'
import { mount } from '@vue/test-utils'
import { createTestingPinia } from '@/tests/testUtils'
import IntegrationStatusCard from '../IntegrationStatusCard.vue'

const mountCard = (status) =>
  mount(IntegrationStatusCard, {
    props: { name: 'BrewGraph', status },
    global: {
      plugins: [createTestingPinia()],
      stubs: { BsCard: { template: '<div><slot /></div>' } }
    }
  })

describe('IntegrationStatusCard', () => {
  it('shows that a configured integration has not run yet', () => {
    expect(mountCard({ push_used: false }).text()).toContain('Not updated yet')
  })

  it('shows a successful last attempt', () => {
    expect(
      mountCard({ push_used: true, push_age: 42000, push_status: true, push_code: 200 }).text()
    ).toContain('Last attempt 42s ago — Success')
  })

  it('shows the last failed HTTP status', () => {
    expect(
      mountCard({ push_used: true, push_age: 60000, push_status: false, push_code: 401 }).text()
    ).toContain('Last attempt 1m ago — Failed (HTTP 401)')
  })
})
