#include <fbs/power.h>
#include <stdio.h>
int main(void) {
    fbs_power_config config = fbs_power_config_default();
    fbs_power_context *context = NULL;
    if (fbs_power_create(&config, NULL, &context) != 0) return 1;
    printf("API version: %u\n", fbs_power_version());
    fbs_power_destroy(context);
    return 0;
}
